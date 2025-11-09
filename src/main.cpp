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
    float f_c_acce = 80.;
    float f_c_gyro = 30.;
    float f_c_magne = 10.;
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

        Quaternion q_desired = {1, 0, 0, 0};
        Quaternion q_e = quat_multiply(q_desired, quat_conjugate(q_drone));
        Vector3f att_e = quat_to_attitude_error(q_e);
        float Kp_att = 4.0f;
        omega_set = vec_scale(att_e, Kp_att);

        debug_counter++;
        if (debug_counter >= 10) {
            debug_counter = 0;
            Serial.printf("%f,%f,%f,%f\n", q_drone.w, q_drone.x, q_drone.y, q_drone.z);
        }
    }

    Vector3f err_ang_rate = vec_sub(omega_set, (Vector3f){tab_gyro[0][0], tab_gyro[1][0], tab_gyro[2][0]});
    float Kp_ang_rate = 50;
    Vector3f torque = vec_scale(err_ang_rate, Kp_ang_rate);

    float base_thrust = 120;
    int PWM_FL = int_clamp(base_thrust - torque.y - torque.x, 0, 255);
    int PWM_FR = int_clamp(base_thrust + torque.y - torque.x, 0, 255);
    int PWM_BR = int_clamp(base_thrust + torque.y + torque.x, 0, 255);
    int PWM_BL = int_clamp(base_thrust - torque.y + torque.x, 0, 255);
    

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