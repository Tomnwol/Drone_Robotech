
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "i2c_basics.h"
#include "MadgwickAHRS.h"
#include "esp_timer.h"
#include "imu.h"
#include "types.h"
#include "utils.h"
//static const char *TAG = "MPU9265";


#define SIZE_TAB  1
float tab_acce[3][SIZE_TAB] = {{0},{0},{0}}; //stocke les dernières valeurs d'accélération
float tab_gyro[3][SIZE_TAB] = {{0},{0},{0}}; //stocke les dernières valeurs du gyro
float tab_magne[3][SIZE_TAB] = {{0},{0},{0}};//stocke les dernières valeurs du mangétomètre

#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_DUTY_RES       LEDC_TIMER_8_BIT // Résolution 8 bits (0-255)
#define LEDC_FREQUENCY      5000              // Fréquence du PWM (5 kHz)

#define LED1_GPIO           16
#define LED2_GPIO           17
#define LED3_GPIO           18
#define LED4_GPIO           19

void hardware_setup(void)
{
    // Configuration du timer PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configuration des canaux PWM
    ledc_channel_config_t ledc_channel[] = {
        {
            .channel    = LEDC_CHANNEL_0,
            .duty       = 0,
            .gpio_num   = LED1_GPIO,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_1,
            .duty       = 0,
            .gpio_num   = LED2_GPIO,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_2,
            .duty       = 0,
            .gpio_num   = LED3_GPIO,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },
        {
            .channel    = LEDC_CHANNEL_3,
            .duty       = 0,
            .gpio_num   = LED4_GPIO,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        }
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }
}

// Exemple de fonction pour allumer les LEDs avec PWM
void set_led_brightness(ledc_channel_t channel, uint32_t duty)
{
    // `duty` va de 0 (éteint) à 255 (pleinement allumé)
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}
void inner_task(void *pvParameter) {
    

    unsigned long lastTime = 0;
    int debug_counter = 0;
    int debug_counter_inner = 0;
    int initialized_gyro = 0;
    int initialized_magne = 0;
    int initialized_accel = 0;

    Quaternion q_drone = {1,0,0,0};
    Vector3f omega_set = {0};

    //Lire magne pas a chaque fois
    Vector3f magne_vect;
    int magne_compteur = 0;
    while (1) { // Inner loop() > 1000Hz idéalement (on obtient ~1070Hz) //Tenter d'atteindre au plus vite la vitesse angulaire demandée par l'outer loop

        Vector3f accel_vect = read_accel();
        Vector3f gyro_vect = read_gyro();
        magne_compteur += 1;
        if (magne_compteur == 10) {
            magne_compteur = 0;
            magne_vect = read_magne();
        }
        
        

        fill_queue(tab_acce[0], SIZE_TAB, accel_vect.x);
        fill_queue(tab_acce[1], SIZE_TAB, accel_vect.y);
        fill_queue(tab_acce[2], SIZE_TAB, accel_vect.z);
        //mean_tab(tab_acce_x, SIZE_TAB);
        /*float w0 = 0.1;
        if (tab_acce[0][1] != 0.0 || initialized_accel == 1){
            initialized_accel = 1;
            tab_acce[0][0] = passe_bas(tab_acce[0][0], tab_acce[0][1], dt, w0);
        }*/
        fill_queue(tab_gyro[0], SIZE_TAB, gyro_vect.x);
        fill_queue(tab_gyro[1], SIZE_TAB, gyro_vect.y);
        fill_queue(tab_gyro[2], SIZE_TAB, gyro_vect.z);
        
        fill_queue(tab_magne[0], SIZE_TAB, magne_vect.x);
        fill_queue(tab_magne[1], SIZE_TAB, magne_vect.y);
        fill_queue(tab_magne[2], SIZE_TAB, magne_vect.z);

        Vector3f average_gyro = {mean_tab(tab_gyro[0], SIZE_TAB), mean_tab(tab_gyro[1], SIZE_TAB), mean_tab(tab_gyro[2], SIZE_TAB)};
        unsigned long now = esp_timer_get_time(); // microsecondes
        float dt = (now - lastTime) / 1000000.0f; // s
        if (dt > 0.004){ //Outer loop | Attitude controle ~500Hz (on obtient ~370Hz)
            lastTime = now;
            float sampleFreqLocal = 1.0f / dt;

            MadgwickAHRSupdate(sampleFreqLocal, average_gyro.x , average_gyro.y, average_gyro.z, mean_tab(tab_acce[0], SIZE_TAB), mean_tab(tab_acce[1], SIZE_TAB), mean_tab(tab_acce[2], SIZE_TAB), mean_tab(tab_magne[0], SIZE_TAB), mean_tab(tab_magne[1], SIZE_TAB), mean_tab(tab_magne[2], SIZE_TAB));   
            
            q_drone.w = q0; q_drone.x =  q1; q_drone.y =  q2; q_drone.z =  q3;
            Quaternion q_desired = {1, 0, 0, 0}; //Orientation voulue (exemple : {0, 0, 0, 1})
            Quaternion q_e = quat_multiply(q_desired, quat_conjugate(q_drone)); //Erreur entre 2 quaternions -> Q_E = Q_D * Q^(-1)
            Vector3f att_e = quat_to_attitude_error(q_e); // Erreur d'attitude approximée
            float Kp_att = 4.0f;  // à ajuster selon la réactivité voulue
            //Correcteur proportionnel sur les rad/s 
            omega_set = vec_scale(att_e, Kp_att); //Régulateur proportionnel | On peut limiter la magnitude si on veut éviter des valeurs aberrantes
            //On obtient une angular rate command en rad/s
            debug_counter += 1;
            if (debug_counter == 10){
                debug_counter = 0;
                printf("%f,%f,%f,%f\n", q_drone.w, q_drone.x, q_drone.y, q_drone.z); // Envoi la donnée 
                //printf("Omega_set:%f,%f,%f\n", omega_set.x, omega_set.y, omega_set.z); 
                //printf("Accel[g]: X=%.2f Y=%.2f Z=%.2f | Gyro[dps]: X=%.2f Y=%.2f Z=%.2f\n", accel_vect.x, accel_vect.y, accel_vect.z, gyro_vect.x, gyro_vect.y, gyro_vect.z);
                //ESP_LOGI(TAG, "XAccel[g]: X=%.2f Y=%.2f Z=%.2f | Gyro[dps]: X=%.2f Y=%.2f Z=%.2f", mean_tab(tab_acce[0], SIZE_TAB), mean_tab(tab_acce[1], SIZE_TAB), mean_tab(tab_acce[2], SIZE_TAB), mean_tab(tab_gyro[0], SIZE_TAB), mean_tab(tab_gyro[1], SIZE_TAB), mean_tab(tab_gyro[2], SIZE_TAB));
                //printf("%.2f,%.2f,%.2f\n",mean_tab(tab_gyro[0], SIZE_TAB), mean_tab(tab_gyro[1], SIZE_TAB), mean_tab(tab_gyro[2], SIZE_TAB));
                //printf("Gyro: X=%.2f Y=%.2f Z=%.2f\n",gyro_vect.x, gyro_vect.y, gyro_vect.z);
                //printf("Magne: X=%.3f Y=%.3f Z=%.3f\n",mean_tab(tab_magne[0], SIZE_TAB), mean_tab(tab_magne[1], SIZE_TAB), mean_tab(tab_magne[2], SIZE_TAB));
                //printf("%.3f,%.3f,%.3f\n",magne_vect.x, magne_vect.y, magne_vect.z);
                //printf("%f,%f,%f\n", error.x, error.y, error.z);
            }
        }
        
        //convertir gyro en rad/s -> OK
        //angular_rate_error = omega_set - gyro_measurement
        Vector3f err_ang_rate = vec_sub(omega_set, average_gyro); // Inverser les 2 composantes ?
        //PID sur ce rate error pour obtenir commande torque_roll, torque_pitch, torque_yaw.
        float Kp_ang_rate = 50;
        Vector3f torque = vec_scale(err_ang_rate, Kp_ang_rate);
        //Controle moteur
        float base_thrust = 120; // PWM moyen pour maintenir altitude
        float roll_cmd  = torque.x;
        float pitch_cmd = torque.y;
        float yaw_cmd   = 0; // Pour l'instant je veux pas commander yaw

        int PWM_FL = (int)(base_thrust - pitch_cmd - roll_cmd + yaw_cmd);
        int PWM_FR = (int)(base_thrust + pitch_cmd - roll_cmd - yaw_cmd);
        int PWM_BR = (int)(base_thrust + pitch_cmd + roll_cmd + yaw_cmd);
        int PWM_BL = (int)(base_thrust - pitch_cmd + roll_cmd - yaw_cmd);
        debug_counter_inner += 1;
        if (debug_counter_inner == 500){
            debug_counter_inner = 0;
            //printf("Omega_set:%f,%f,%f\n", omega_set.x, omega_set.y, omega_set.z); 
            //printf("ave_gyro:%f,%f,%f\n", average_gyro.x, average_gyro.y, average_gyro.z);
            //printf("torque:%f,%f,%f\n", torque.x, torque.y, torque.z);
        }
        // Limitation PWM
        PWM_FR = int_clamp(PWM_FR, 0, 255);
        PWM_FL = int_clamp(PWM_FL, 0, 255);
        PWM_BR = int_clamp(PWM_BR, 0, 255);
        PWM_BL = int_clamp(PWM_BL, 0, 255);

        set_led_brightness(LEDC_CHANNEL_2, PWM_FR);
        set_led_brightness(LEDC_CHANNEL_1, PWM_FL);
        set_led_brightness(LEDC_CHANNEL_3, PWM_BR);
        set_led_brightness(LEDC_CHANNEL_0, PWM_BL);
    }
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE); // désac les ESP LOGI
    i2c_master_init(I2C_NUM_0, I2C_MASTER_0_SDA_IO, I2C_MASTER_0_SCL_IO, I2C_MASTER_0_FREQ_HZ);
    i2c_master_init(I2C_NUM_1, I2C_MASTER_1_SDA_IO, I2C_MASTER_1_SCL_IO, I2C_MASTER_1_FREQ_HZ);
    qmc5883l_init();
    hardware_setup();
    // Wake up MPU9265
    i2c_write_byte(I2C_MASTER_0_PORT, MPU9265_ADDR, MPU9265_PWR_MGMT_1_REG, 0x00);
    calibrate_gyro();
    // Tâches FreeRTOS
    xTaskCreate(inner_task, "inner_task", 2048, NULL, 10, NULL);
    xTaskCreate(outer_task, "outer_task", 4096, NULL, 5, NULL);
}
