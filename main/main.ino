//#include "Arduino.h"
#include <stdio.h>
#include <Arduino.h>
#include <DShotRMT.h>
#include "i2c_basics.hpp"
#include "MadgwickAHRS.hpp"
#include "esp_timer.h"
#include "esp_log.h"
#include "imu.hpp"
#include "types.hpp"
#include "utils.hpp"
#include "UDP_reception.hpp"
//#include "NRF_receiver.hpp"
#include "pid.hpp"

static constexpr gpio_num_t MOTOR_FL_PIN = GPIO_NUM_13; //MOT1
static constexpr gpio_num_t MOTOR_FR_PIN = GPIO_NUM_26;//GPIO_NUM_32; //MOT3
static constexpr gpio_num_t MOTOR_BL_PIN = GPIO_NUM_27;//GPIO_NUM_14; //MOT2
static constexpr gpio_num_t MOTOR_BR_PIN = GPIO_NUM_33; //MOT4
static constexpr gpio_num_t ARM_LED = GPIO_NUM_12; //HIGH state indicates that the drone is ready to fly
static constexpr gpio_num_t GP_LED = GPIO_NUM_15;
static constexpr dshot_mode_t MODE = DSHOT600;
DShotRMT motorFL(MOTOR_FL_PIN, MODE, false);
DShotRMT motorFR(MOTOR_FR_PIN, MODE, false);
DShotRMT motorBL(MOTOR_BL_PIN, MODE, false);
DShotRMT motorBR(MOTOR_BR_PIN, MODE, false);

#define TIME_BEFORE_MOTOR_ACTIVATION 20000000
#define BASE_THRUST_MIN 100
#define BASE_THRUST_MAX 1900
#define BASE_THRUST_MAX_FAILSAFE 700
#define BASE_THRUST_FAILSAFE_AUTO 400

#define MOTOR_MAX_VALUE 2047
#define MOTOR_FAILSAFE_MAX_VALUE 1000

#define MOTOR_SLEW_RATE 2000 //DSHOT units/s 
#define OFFSET_MOTOR_MAX 100
int failsafe_mode = 0; /* 0->OK  1->Activation FS par radio  2->Communication perdue */
bool killswitch_enable = false; /* Si le mode KS est activé, il ne pourra plus être désactivé avant redémarrage */ 
bool is_arm_mode_enable = false;
#define SIZE_TAB  2
float tab_acce[3][SIZE_TAB] = {{0},{0},{0}}; //stocke les dernières valeurs d'accélération
float tab_gyro[3][SIZE_TAB] = {{0},{0},{0}}; //stocke les dernières valeurs du gyro
float tab_magne[3][SIZE_TAB] = {{0},{0},{0}}; //stocke les dernières valeurs du mangétomètre

bool I2C_init_error = false;
void fill_queues(Vector3f accel_vect, Vector3f gyro_vect, Vector3f magne_vect){
    fill_queue(tab_acce[0], SIZE_TAB, accel_vect.x);
    fill_queue(tab_acce[1], SIZE_TAB, accel_vect.y);
    fill_queue(tab_acce[2], SIZE_TAB, accel_vect.z);
    fill_queue(tab_gyro[0], SIZE_TAB, gyro_vect.x);
    fill_queue(tab_gyro[1], SIZE_TAB, gyro_vect.y);
    fill_queue(tab_gyro[2], SIZE_TAB, gyro_vect.z);
    fill_queue(tab_magne[0], SIZE_TAB, magne_vect.x);
    fill_queue(tab_magne[1], SIZE_TAB, magne_vect.y);
    fill_queue(tab_magne[2], SIZE_TAB, magne_vect.z);
}

void passes_bas(float dt){
    float f_c_acce = 60.;
    float f_c_gyro = 130.;
    float f_c_magne = 50.;
    tab_acce[0][0] = passe_bas(tab_acce[0][0], tab_acce[0][1], dt, f_c_acce);
    tab_acce[1][0] = passe_bas(tab_acce[1][0], tab_acce[1][1], dt, f_c_acce);
    tab_acce[2][0] = passe_bas(tab_acce[2][0], tab_acce[2][1], dt, f_c_acce);

    tab_gyro[0][0] = passe_bas(tab_gyro[0][0], tab_gyro[0][1], dt, f_c_gyro);
    tab_gyro[1][0] = passe_bas(tab_gyro[1][0], tab_gyro[1][1], dt, f_c_gyro);
    tab_gyro[2][0] = passe_bas(tab_gyro[2][0], tab_gyro[2][1], dt, f_c_gyro);

    tab_magne[0][0] = passe_bas(tab_magne[0][0], tab_magne[0][1], dt, f_c_magne);
    tab_magne[1][0] = passe_bas(tab_magne[1][0], tab_magne[1][1], dt, f_c_magne);
    tab_magne[2][0] = passe_bas(tab_magne[2][0], tab_magne[2][1], dt, f_c_magne);
}


float get_base_thrust_max(){
    if (failsafe_mode){
      return BASE_THRUST_MAX_FAILSAFE;
    }else{
      return BASE_THRUST_MAX;
    }
}

float get_motor_max_value(){
    if (failsafe_mode){
      return MOTOR_FAILSAFE_MAX_VALUE;
    }else{
      return MOTOR_MAX_VALUE;
    }
}

float prev_mFL = 48.0f;
float prev_mFR = 48.0f;
float prev_mBR = 48.0f;
float prev_mBL = 48.0f;
void motors_zeroes(){
  prev_mFL = 48.0f; prev_mFR = 48.0f; prev_mBR = 48.0f; prev_mBL = 48.0f;
  motorFL.sendThrottle(48);
  motorFR.sendThrottle(48);
  motorBL.sendThrottle(48);
  motorBR.sendThrottle(48);
}

void motor_initial_sequence(){
  unsigned long start = millis();
  while (millis() - start < 1000) {
    motorFL.sendThrottle(0);
    motorFR.sendThrottle(0);
    motorBL.sendThrottle(0);
    motorBR.sendThrottle(0);
    delay(2);
  }
  delay(500);
  for (uint16_t t = 48; t <= 200; ++t) {
    motorFL.sendThrottle(t);
    motorFR.sendThrottle(t);
    motorBL.sendThrottle(t);
    motorBR.sendThrottle(t);
    delay(5); // ramp lente
  }
  // Retour à zéro
  for (uint16_t t = 200; t >= 48; --t) {
    motorFL.sendThrottle(t);
    motorFR.sendThrottle(t);
    motorBL.sendThrottle(t);
    motorBR.sendThrottle(t);
    delay(5);
  }
}

bool arm_mode_enable(){
  static int arming_counter = 0;
  if (joyThrottle == 48 && udp_delta_time < 100){
    arming_counter++;
    if (arming_counter >= 10){
      return true;
    }
  }else{
    arming_counter = 0;
  }
  return false;
}

bool is_kill_mode_enable(){ /* Non réversible mode -> désactivation des moteurs */
  if (killswitch_enable == false){
    if (is_arm_mode_enable && (udp_delta_time > 1000)){ //1500 ! Pour les TESTS ON MET + MAIS ATTENTION A BIEN REMETTRE 1500 PLUS TARD
      killswitch_enable = true;
      return true;
    }
    if (SWKillSwitch){ // Si la radio a reçu un signal KillSwitch
      killswitch_enable = true;
      return true;
    }else{
      return false;
    }
    return false;
  }else{
    return true;
  }
}

void failsafe_mode_activation(){  /* 0->OK  1->Activation par radio  2->Communication perdue */
  if (is_arm_mode_enable && udp_delta_time > 500){ // La radio ne répond plus depuis 0.5s => failsafe mode => limiation moteur
    failsafe_mode = 2;
    return;
  }
  failsafe_mode = SWFailSafe;
}

void error_loop_blink(){
  while(1){
    delay(200);
    Serial.println("KS_Activated");
    digitalWrite(GP_LED, HIGH);
    delay(200);
    digitalWrite(GP_LED, LOW);
  }
}

void fc_task() { 
    static int debug_counter_inner = 0;
    static unsigned long lastTime_outter = 0;
    static unsigned long lastTime_inner = 0;
    static int debug_counter = 0;
    static int debug_counter2 = 0;
    debug_counter_inner++;
    debug_counter2++;

    static int magne_compteur = 0;
    static Quaternion q_drone = {1, 0, 0, 0};
    static Vector3f omega_set = {0};
    static Vector3f magne_vect = {0, 0, 0};
    static Euler att_telemetry = {0,0,0};
    static uint16_t MOT_FL_telemetry = 48;
    static uint16_t MOT_FR_telemetry = 48;
    static uint16_t MOT_BR_telemetry = 48;
    static uint16_t MOT_BL_telemetry = 48;

    handleSendTelemetry(att_telemetry, MOT_FL_telemetry, MOT_FR_telemetry, MOT_BR_telemetry, MOT_BL_telemetry);

    unsigned long now = micros(); // microsecondes
    float dt_inner = (now - lastTime_inner) / 1000000.0f; // s [Si bug -> Vérifier que dt_inner ne vaut jamais 0]
    lastTime_inner = now;

    // Lecture capteurs
    Vector3f accel_vect = read_accel();
    Vector3f gyro_vect  = read_gyro();
    magne_compteur++;
    fill_queues(accel_vect, gyro_vect, magne_vect);
    passes_bas(dt_inner);

    float dt_outer = (now - lastTime_outter) / 1000000.0f; // s
    /* OUTER LOOP */
    if (dt_outer > 0.004f) { // ~250 Hz
        lastTime_outter = now;
        float sampleFreqLocal = 1.0f / dt_outer;
        readUDPData();

        MadgwickAHRSupdate(
            sampleFreqLocal,
            tab_gyro[0][0], tab_gyro[1][0], tab_gyro[2][0],
            tab_acce[0][0], tab_acce[1][0], tab_acce[2][0],
            //tab_magne[0][0], tab_magne[1][0], tab_magne[2][0]
            0, 0, 0
        );
        q_drone.w = q0; q_drone.x = q1; q_drone.y = q2; q_drone.z = q3;
      
        Euler att = quat_to_euler(q_drone); // Obtiens l'attitude en Euler
        att_telemetry = att;
        const float SECURITY_ANGLE_RAD = 45.0f * M_PI / 180.0f;
        if (now > TIME_BEFORE_MOTOR_ACTIVATION * 0.8 && (fabs(att.roll) > SECURITY_ANGLE_RAD || fabs(att.pitch) > SECURITY_ANGLE_RAD)){
          killswitch_enable = true;
        }

        Euler att_desired = get_att_desired();
        //Euler att_desired = {0.0f, 0.0f, 0.0f}; // attitude cible (par ex. à plat)
        omega_set = get_angular_rate_command(att, att_desired);
    }

    Vector3f torque = get_torque(omega_set, (Vector3f){tab_gyro[0][0], tab_gyro[1][0], tab_gyro[2][0]}, (Vector3f){tab_gyro[0][1], tab_gyro[1][1], tab_gyro[2][1]}, dt_inner);
    
    if (killswitch_enable){
      error_loop_blink();
    }
    bool motor_zero = false;
    if(now < TIME_BEFORE_MOTOR_ACTIVATION * 0.9 ){
      motor_zero = true;
    }
    digitalWrite(GP_LED, HIGH);
    // Vérification de l'armement du drone. Une fois que le drone est armé, ce n'est plus vérfié
    if( !is_arm_mode_enable ){ 
      is_arm_mode_enable = arm_mode_enable();
      if (debug_counter2 >= 200) {
        debug_counter2 = 0;
        Serial.printf("%d, %d\n", joyThrottle, udp_delta_time);
      }
      if (is_arm_mode_enable){ //La LED prévient que le drone a été armé
        digitalWrite(ARM_LED, HIGH);
        //Ce serait mieux d'armer les moteurs ici, cependant, => Problématique en communication unidirectionnelle si un moteur failli ( et si on laisse une séquence de démarrage tout ça, après on a le problème de l'algo de Madwick qui se perd)
      }
      motor_zero = true;
    }
    
    failsafe_mode_activation();
    float base_thrust_max = get_base_thrust_max();
    float base_thrust;
    if (failsafe_mode != 2){
      base_thrust = (float)joyThrottle;
    }else{ // FS 2 -> Communication perdue
      base_thrust = BASE_THRUST_FAILSAFE_AUTO;
    }

    base_thrust = float_clamp(base_thrust, BASE_THRUST_MIN, base_thrust_max);


    if(now < TIME_BEFORE_MOTOR_ACTIVATION || is_kill_mode_enable()){ // ATTENTION A BIEN REGLER LA VALEUR DE DELTA RADIO LORS DUN VRAI VOL
      motor_zero = true;
    }
    float motor_max_value = get_motor_max_value();
    // Remarque: sur ce montage, l'IMU est orientée de 90°
    // => gyro.x = roulis (roll), gyro.y = tangage (pitch)
    //On peut viser : torque.x, torque.y ∈ [-100, +100] & base_thrust ∈ [100 à 1900]
    float mFL_target = float_clamp((base_thrust + torque.y - torque.x + uint8_clamp(offsetMotorFL, 0, OFFSET_MOTOR_MAX) ), 48.0, motor_max_value);
    float mFR_target = float_clamp((base_thrust + torque.y + torque.x + uint8_clamp(offsetMotorFR, 0, OFFSET_MOTOR_MAX) ), 48.0, motor_max_value);
    float mBR_target = float_clamp((base_thrust - torque.y + torque.x + uint8_clamp(offsetMotorBR, 0, OFFSET_MOTOR_MAX) ), 48.0, motor_max_value);
    float mBL_target = float_clamp((base_thrust - torque.y - torque.x + uint8_clamp(offsetMotorBL, 0, OFFSET_MOTOR_MAX) ), 48.0, motor_max_value);

    // Appliquer slew limiter par moteur
    prev_mFL = slew_limit(mFL_target, prev_mFL, MOTOR_SLEW_RATE, dt_inner);
    prev_mFR = slew_limit(mFR_target, prev_mFR, MOTOR_SLEW_RATE, dt_inner);
    prev_mBR = slew_limit(mBR_target, prev_mBR, MOTOR_SLEW_RATE, dt_inner);
    prev_mBL = slew_limit(mBL_target, prev_mBL, MOTOR_SLEW_RATE, dt_inner);

    uint16_t MOT_FL = (uint16_t)float_clamp(prev_mFL, 48.0, motor_max_value);
    uint16_t MOT_FR = (uint16_t)float_clamp(prev_mFR, 48.0, motor_max_value);
    uint16_t MOT_BR = (uint16_t)float_clamp(prev_mBR, 48.0, motor_max_value);
    uint16_t MOT_BL = (uint16_t)float_clamp(prev_mBL, 48.0, motor_max_value);
    MOT_FL_telemetry = MOT_FL;
    MOT_FR_telemetry = MOT_FR;
    MOT_BR_telemetry = MOT_BR;
    MOT_BL_telemetry = MOT_BL;

    if(!motor_zero){
      motorFL.sendThrottle(MOT_FL);
      motorFR.sendThrottle(MOT_FR);
      motorBL.sendThrottle(MOT_BL);
      motorBR.sendThrottle(MOT_BR);
    }else{
      motors_zeroes();
    }

    debug_counter++;
    if (debug_counter >= 200) {
        debug_counter = 0;
        //Afficher également les commandes Moteur1,2,3,4////////////////////////////////////////////////////
        Serial.printf("%f,%f,%f,%f,%d,%d,%d,%d\n", q_drone.w, q_drone.x, q_drone.y, q_drone.z,MOT_FL,MOT_FR,MOT_BL,MOT_BR );
        
    }
}



void setup() {
    Serial.begin(115200);
    pinMode(ARM_LED, OUTPUT);
    pinMode(GP_LED, OUTPUT);
    pinMode(BATTERY_PIN, INPUT);
    digitalWrite(GP_LED, LOW);
    digitalWrite(ARM_LED, LOW);
    i2c_master_init(I2C_MASTER_0_PORT, I2C_MASTER_0_SDA_IO, I2C_MASTER_0_SCL_IO, I2C_MASTER_0_FREQ_HZ);
    calibrate_gyro();
    delay(100);
    motorFL.begin();
    motorFR.begin();
    motorBL.begin();
    motorBR.begin();
    motor_initial_sequence();
    delay(50); //JTAG Delay test
    setupUDP();
}


void loop() {
    fc_task();
}
