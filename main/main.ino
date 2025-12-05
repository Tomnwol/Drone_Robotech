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
#include "NRF_receiver.hpp"
#include "pid.hpp"

static constexpr gpio_num_t MOTOR_FL_PIN = GPIO_NUM_13; //MOT1
static constexpr gpio_num_t MOTOR_FR_PIN = GPIO_NUM_26;//GPIO_NUM_32; //MOT3
static constexpr gpio_num_t MOTOR_BL_PIN = GPIO_NUM_27;//GPIO_NUM_14; //MOT2
static constexpr gpio_num_t MOTOR_BR_PIN = GPIO_NUM_33; //MOT4
static constexpr dshot_mode_t MODE = DSHOT600;
DShotRMT motorFL(MOTOR_FL_PIN, MODE, false);
DShotRMT motorFR(MOTOR_FR_PIN, MODE, false);
DShotRMT motorBL(MOTOR_BL_PIN, MODE, false);
DShotRMT motorBR(MOTOR_BR_PIN, MODE, false);

#define BASE_THRUST_MIN 100
#define BASE_THRUST_MAX 1900
#define BASE_THRUST_MAX_FAILSAFE 700
#define BASE_THRUST_FAILSAFE_AUTO 600
int failsafe_mode = 0; /* 0->OK  1->Activation FS par radio  2->Communication perdue */
bool killswitch_enable = false; /* Si le mode KS est activé, il ne pourra plus être désactivé avant redémarrage */ 

#define SIZE_TAB  2
float tab_acce[3][SIZE_TAB] = {{0},{0},{0}}; //stocke les dernières valeurs d'accélération
float tab_gyro[3][SIZE_TAB] = {{0},{0},{0}}; //stocke les dernières valeurs du gyro
float tab_magne[3][SIZE_TAB] = {{0},{0},{0}};//stocke les dernières valeurs du mangétomètre

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
    float f_c_acce = 280.;
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
      return BASE_THRUST_MAX;
    }else{
      return BASE_THRUST_MAX_FAILSAFE;
    }
}

bool is_kill_mode_enable(){ /* Non réversible mode -> désactivation des moteurs */
  if (killswitch_enable == false){
    if (radio_delta_time > 1500){
      killswitch_enable = true;
      return true;
    }
    if (SWKillSwitch){ // Si la radio a reçu un signal KillSwitch
      killswitch_enable = true;
      return true;
    }else{
      return false;
    }
  }else{
    return true;
  }
}

void failsafe_mode_activation(){  /* 0->OK  1->Activation par radio  2->Communication perdue */
  if (radio_delta_time > 500){ // La radio ne répond plus depuis 0.5s => failsafe mode => limiation moteur
    failsafe_mode = 2;
    return;
  }
  failsafe_mode = SWFailSafe;
}

void fc_task() { 
    static int debug_counter_inner = 0;
    static unsigned long lastTime_outter = 0;
    static unsigned long lastTime_inner = 0;
    static int debug_counter = 0;
    
    debug_counter_inner++;
    if (debug_counter_inner >= 500) {
        debug_counter_inner = 0;
       // Ajouter une Sécu, si pas de commande pendant x ms OU valeurs incohérentes => Arret obligatoire
    }

    /*if (I2C_init_error == true){ //Cette sécurité ne marche pas, à vérifier
        return;
    }*/
    static int magne_compteur = 0;
    static Quaternion q_drone = {1, 0, 0, 0};
    static Vector3f omega_set = {0};
    static Vector3f magne_vect = {0, 0, 0};

    unsigned long now = micros(); // microsecondes
    float dt_inner = (now - lastTime_inner) / 1000000.0f; // s
    lastTime_inner = now;

    // Lecture capteurs
    Vector3f accel_vect = read_accel();
    Vector3f gyro_vect  = read_gyro();

    magne_compteur++;
    /*if (magne_compteur >= 10) {
        magne_compteur = 0;
        magne_vect = read_magne();
    }*/

    fill_queues(accel_vect, gyro_vect, magne_vect);
    passes_bas(dt_inner);

    float dt_outer = (now - lastTime_outter) / 1000000.0f; // s
    if (dt_outer > 0.004f) { // ~250 Hz
        lastTime_outter = now;
        float sampleFreqLocal = 1.0f / dt_outer;
        //readNRFData();

        MadgwickAHRSupdate(
            sampleFreqLocal,
            tab_gyro[0][0], tab_gyro[1][0], tab_gyro[2][0],
            tab_acce[0][0], tab_acce[1][0], tab_acce[2][0],
            //tab_magne[0][0], tab_magne[1][0], tab_magne[2][0]
            0, 0, 0
        );

        q_drone.w = q0; q_drone.x = q1; q_drone.y = q2; q_drone.z = q3;
      
        Euler att = quat_to_euler(q_drone); // Obtiens l'attitude en Euler
        const float MAX_ANGLE_RAD = 45.0f * M_PI / 180.0f;
        if (fabs(att.roll) > MAX_ANGLE_RAD || fabs(att.pitch) > MAX_ANGLE_RAD){
          killswitch_enable = true;
        }
        Euler att_desired = {0.0f, 0.0f, 0.0f}; // attitude cible (par ex. à plat)

        omega_set = get_angular_rate_command(att, att_desired);
    }

    Vector3f torque = get_torque(omega_set, (Vector3f){tab_gyro[0][0], tab_gyro[1][0], tab_gyro[2][0]}, dt_inner);

    failsafe_mode_activation();
    float base_thrust_max = get_base_thrust_max();
    float base_thrust;
    if (failsafe_mode != 2){
      base_thrust = float_remap((float)joyThrottle, 0, 1023, BASE_THRUST_MIN, base_thrust_max); //Récupérer la valeur de la télécommande
    }else{ // FS 2 -> Communication perdue
      base_thrust = float_remap(BASE_THRUST_FAILSAFE_AUTO, 0, 1023, BASE_THRUST_MIN, base_thrust_max);
    }

    base_thrust = float_clamp(base_thrust, BASE_THRUST_MIN, BASE_THRUST_MAX);

    // Remarque: sur ce montage, l'IMU est orientée de 90°
    // => gyro.x = roulis (roll), gyro.y = tangage (pitch)
    //On peut viser : torque.x, torque.y ∈ [-500, +500] & base_thrust ∈ [100 à 1900]
    if(!is_kill_mode_enable()){
      
      uint16_t MOT_FL = int_clamp(base_thrust - torque.y + torque.x, 48, 2047);
      uint16_t MOT_FR = int_clamp(base_thrust - torque.y - torque.x, 48, 2047);
      uint16_t MOT_BR = int_clamp(base_thrust + torque.y - torque.x, 48, 2047);
      uint16_t MOT_BL = int_clamp(base_thrust + torque.y + torque.x, 48, 2047);
      motorFL.sendThrottle(MOT_FL);
      motorFR.sendThrottle(MOT_FR);
      motorBL.sendThrottle(MOT_BR);
      motorBR.sendThrottle(MOT_BL);
      debug_counter++;
      if (debug_counter >= 100) {
          debug_counter = 0;
          //Afficher également les commandes Moteur1,2,3,4////////////////////////////////////////////////////
          Serial.printf("%f,%f,%f,%f,%d,%d,%d,%d\n", q_drone.w, q_drone.x, q_drone.y, q_drone.z,MOT_FL,MOT_FR,MOT_BL,MOT_BR );
      }
    }
}

void setup() {
    Serial.begin(115200);
    i2c_master_init(I2C_MASTER_0_PORT, I2C_MASTER_0_SDA_IO, I2C_MASTER_0_SCL_IO, I2C_MASTER_0_FREQ_HZ);
    //i2c_master_init(I2C_MASTER_1_PORT, I2C_MASTER_1_SDA_IO, I2C_MASTER_1_SCL_IO, I2C_MASTER_1_FREQ_HZ);
    //qmc5883l_init();
    setupNRF();
    //bool is_magne_present = isMagnePresent();
    //bool is_MPU_present = isMPUPresent();
    //I2C_init_error = !is_magne_present || !is_MPU_present;
    delay(500); //JTAG Delay test
    motorFL.begin();
    motorFR.begin();
    motorBL.begin();
    motorBR.begin();
    delay(2);
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
    //setupNRF();
}


void loop() {
    //fc_task(); // Remplace la version FreeRTOS par une fonction Arduino
}