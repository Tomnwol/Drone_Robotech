//#include "Arduino.h"
#include <stdio.h>
#include <Arduino.h>
#include "i2c_basics.hpp"
#include "MadgwickAHRS.hpp"
#include "esp_timer.h"
#include "esp_log.h"
#include "imu.hpp"
#include "types.hpp"
#include "utils.hpp"
#include "NRF_receiver.hpp"
//static const char *TAG = "MPU9265";

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
Quaternion remove_yaw(Quaternion q) {
    // Extraire les angles Euler
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    float roll = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2 * (q.w * q.y - q.z * q.x);
    float pitch = fabs(sinp) >= 1 ? copysign(M_PI / 2, sinp) : asin(sinp);

    // Ignorer le yaw (on le remet à 0)
    float yaw = 0.0f;

    // Recréer un quaternion sans yaw
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    Quaternion q_no_yaw;
    q_no_yaw.w = cr * cp * cy + sr * sp * sy;
    q_no_yaw.x = sr * cp * cy - cr * sp * sy;
    q_no_yaw.y = cr * sp * cy + sr * cp * sy;
    q_no_yaw.z = cr * cp * sy - sr * sp * cy;
    return q_no_yaw;
}

struct Euler {
  float roll;
  float pitch;
  float yaw;
};

Euler quat_to_euler(Quaternion q) {
    Euler e;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    e.roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f)
        e.pitch = copysignf(M_PI / 2.0f, sinp); // use 90° if out of range
    else
        e.pitch = asinf(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    e.yaw = atan2f(siny_cosp, cosy_cosp);

    return e;
}


void fc_task() {

    
    static int debug_counter_inner = 0;
    static unsigned long lastTime_outter = 0;
    static unsigned long lastTime_inner = 0;
    static int debug_counter = 0;
    
    debug_counter_inner++;
    if (debug_counter_inner >= 500) {
        debug_counter_inner = 0;
        readNRFData();
    }

    if (I2C_init_error == true){
        return;
    }
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
    if (magne_compteur >= 10) {
        magne_compteur = 0;
        magne_vect = read_magne();
    }

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
            tab_magne[0][0], tab_magne[1][0], tab_magne[2][0]
        );

        q_drone.w = q0; q_drone.x = q1; q_drone.y = q2; q_drone.z = q3;
        
        /*
        Quaternion q_desired = {1, 0, 0, 0};
        Quaternion q_e = quat_multiply(q_desired, quat_conjugate(q_drone));
        Vector3f att_e = quat_to_attitude_error(q_e);
        
        float Kp_att = 4.0f;
        omega_set = vec_scale(att_e, Kp_att);
        */
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
    float Kp_ang_rate = 50; //Mettre un Kp plus faible, voir les valeurs min et max de variation, puis remap pour le dshot
    Vector3f torque = vec_scale(err_ang_rate, Kp_ang_rate);

    float base_thrust = 1000;
    // Remarque: sur ce montage, l'IMU est orientée de 90°
    // => gyro.x = roulis (roll), gyro.y = tangage (pitch)
    // Mixage adapté 
    uint16_t MOT_FL = int_clamp(base_thrust - torque.y + torque.x, 48, 2047);
    uint16_t MOT_FR = int_clamp(base_thrust - torque.y - torque.x, 48, 2047);
    uint16_t MOT_BR = int_clamp(base_thrust + torque.y - torque.x, 48, 2047);
    uint16_t MOT_BL = int_clamp(base_thrust + torque.y + torque.x, 48, 2047);
    debug_counter++;
    if (debug_counter >= 100) {
        debug_counter = 0;
        //Afficher également les commandes Moteur1,2,3,4////////////////////////////////////////////////////
        Serial.printf("%f,%f,%f,%f,%d,%d,%d,%d\n", q_drone.w, q_drone.x, q_drone.y, q_drone.z,MOT_FL,MOT_FR,MOT_BL,MOT_BR );
        
    }
}

void setup() {
    Serial.begin(115200);
    i2c_master_init(I2C_MASTER_0_PORT, I2C_MASTER_0_SDA_IO, I2C_MASTER_0_SCL_IO, I2C_MASTER_0_FREQ_HZ);
    i2c_master_init(I2C_MASTER_1_PORT, I2C_MASTER_1_SDA_IO, I2C_MASTER_1_SCL_IO, I2C_MASTER_1_FREQ_HZ);
    qmc5883l_init();
    bool is_magne_present = isMagnePresent();
    bool is_MPU_present = isMPUPresent();
    I2C_init_error = !is_magne_present || !is_MPU_present;
    setupNRF();
    Serial.println("DebugTestprint");
    
}


void loop() {
    fc_task(); // Remplace la version FreeRTOS par une fonction Arduino
}