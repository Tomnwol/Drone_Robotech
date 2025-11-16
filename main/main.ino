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

static constexpr gpio_num_t MOTOR_PIN = GPIO_NUM_13;
static constexpr dshot_mode_t MODE = DSHOT600;
DShotRMT motor(MOTOR_PIN, MODE, false);

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

void fc_task() { 
    static int debug_counter_inner = 0;
    static unsigned long lastTime_outter = 0;
    static unsigned long lastTime_inner = 0;
    static int debug_counter = 0;
    
    debug_counter_inner++;
    if (debug_counter_inner >= 500) {
        debug_counter_inner = 0;
       readNRFData();// Ajouter une Sécu, si pas de commande pendant x ms OU valeurs incohérentes => Arret obligatoire
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

        MadgwickAHRSupdate(
            sampleFreqLocal,
            tab_gyro[0][0], tab_gyro[1][0], tab_gyro[2][0],
            tab_acce[0][0], tab_acce[1][0], tab_acce[2][0],
            //tab_magne[0][0], tab_magne[1][0], tab_magne[2][0]
            0, 0, 0
        );

        q_drone.w = q0; q_drone.x = q1; q_drone.y = q2; q_drone.z = q3;
      
        // Obtiens les angles
        Euler att = quat_to_euler(q_drone);

        // Définis ton attitude cible (par ex. à plat)
        Euler att_desired = {0.0f, 0.0f, 0.0f};

        // Calcul d’erreur
        float err_roll  = att_desired.roll  - att.roll;
        float err_pitch = att_desired.pitch - att.pitch;

        // (Optionnel) ignorer le yaw car pas mesuré correctement
        float err_yaw = 0.0f;

        // PID ou proportionnel simple
        float Kp_att = 4.0f;
        float omega_roll  = Kp_att * err_roll;
        float omega_pitch = Kp_att * err_pitch;

        // Tu veux ensuite obtenir ton couple cible (en vitesse angulaire)
        omega_set = {omega_roll, omega_pitch, 0.0f};

        
    }

    Vector3f err_ang_rate = vec_sub(omega_set, (Vector3f){tab_gyro[0][0], tab_gyro[1][0], tab_gyro[2][0]});
    
    float Kp_ang_rate = 5; //Mettre un Kp plus faible, voir les valeurs min et max de variation, puis remap pour le dshot
    float Ki_ang_rate = 0.5f;
    Vector3f integration_min = {-0.5, -0.5, 0};  
    Vector3f integration_max = {0.5, 0.5, 0};      // Limite pour l'intégrateur (à ajuster selon essais)
    static Vector3f err_ang_rate_int = {0, 0, 0};
    // Accumulation de l'intégrateur
    err_ang_rate_int = vec_add( err_ang_rate_int , vec_scale(err_ang_rate, dt_inner) );
    // Saturation de l’intégrateur (anti-windup)
    err_ang_rate_int = vec_clamp(err_ang_rate_int, integration_min, integration_max);

    Vector3f torque = vec_add(
        vec_scale(err_ang_rate, Kp_ang_rate),
        vec_scale(err_ang_rate_int, Ki_ang_rate)
    );
    //if (debug_counter ==0) {
    //    Serial.printf("torque : %f,%f\n", torque.x, torque.y);
    //}
    float torque_amplitude = 50.0; // valeur d'amplitude empirique mesurée avec Kp = 5 et Ki = 0.5
    torque = vec_scale(torque, 500.0f/torque_amplitude); // On veut un torque entre -500 et 500 
    torque = vec_clamp(torque, (Vector3f){-500,-500,0}, (Vector3f){500,500,0});
    float base_thrust = 1000; //Récupérer la valeur de la télécommande
    float base_thrust_min = 100;
    float base_thrust_max = 1900;
    base_thrust = float_clamp(base_thrust, base_thrust_min, base_thrust_max);
    // Remarque: sur ce montage, l'IMU est orientée de 90°
    // => gyro.x = roulis (roll), gyro.y = tangage (pitch)
    //On peut viser : torque.x, torque.y ∈ [-500, +500] & base_thrust ∈ [100 à 1900]
    uint16_t MOT_FL = int_clamp(base_thrust - torque.y + torque.x, 48, 2047);
    uint16_t MOT_FR = int_clamp(base_thrust - torque.y - torque.x, 48, 2047);
    uint16_t MOT_BR = int_clamp(base_thrust + torque.y - torque.x, 48, 2047);
    uint16_t MOT_BL = int_clamp(base_thrust + torque.y + torque.x, 48, 2047);

    debug_counter++;
    if (debug_counter >= 100) {
        debug_counter = 0;
        //Afficher également les commandes Moteur1,2,3,4////////////////////////////////////////////////////
        //Serial.printf("%f,%f,%f,%f,%d,%d,%d,%d\n", q_drone.w, q_drone.x, q_drone.y, q_drone.z,MOT_FL,MOT_FR,MOT_BL,MOT_BR );
        
    }
}

void setup() {
    Serial.begin(115200);
    i2c_master_init(I2C_MASTER_0_PORT, I2C_MASTER_0_SDA_IO, I2C_MASTER_0_SCL_IO, I2C_MASTER_0_FREQ_HZ);
    i2c_master_init(I2C_MASTER_1_PORT, I2C_MASTER_1_SDA_IO, I2C_MASTER_1_SCL_IO, I2C_MASTER_1_FREQ_HZ);
    qmc5883l_init();
    //bool is_magne_present = isMagnePresent();
    //bool is_MPU_present = isMPUPresent();
    //I2C_init_error = !is_magne_present || !is_MPU_present;
    
    motor.begin();
    /*unsigned long start = millis();
    while (millis() - start < 1000) {
      motor.sendThrottle(0);
      delay(2);
    }
    delay(200);
    for (uint16_t t = 48; t <= 200; ++t) {
      motor.sendThrottle(t);
      delay(50); // ramp lente
    }
    // Retour à zéro
    for (uint16_t t = 200; t >= 48; --t) {
      motor.sendThrottle(t);
      delay(25);
    }*/
    setupNRF();
}


void loop() {
    fc_task(); // Remplace la version FreeRTOS par une fonction Arduino
}