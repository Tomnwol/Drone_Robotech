
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "i2c_basics.h"
#include "MadgwickAHRS.h"
#include "esp_timer.h"
#include "imu.h"
#include "types.h"
static const char *TAG = "MPU9265";


#define SIZE_TAB  16
float tab_acce[3][SIZE_TAB]; //stocke les dernières valeurs d'accélération
float tab_gyro[3][SIZE_TAB]; //stocke les dernières valeurs du gyro
float tab_magne[3][SIZE_TAB];//stocke les dernières valeurs du mangétomètre

void fill_tab(float * tab, int size_tab){
    int i = 0;
    for (i=0; i < size_tab;i++){
        tab[i] = 0;
    }
}

float mean_tab(float * tab, int size_tab){  // Moyenne quadratique pondérée | à remplacer par un filtre passe bas?
    float tot = 0;
    int coef_sum = 0;
    for(int i = 0; i < size_tab;i++){
        tot +=    (size_tab-i)*(size_tab-i)*tab[i];
        coef_sum+=(size_tab-i)*(size_tab-i);
    }
    float moy = tot / coef_sum;
    return(moy);
}


void fill_queue(float * tab, int size_tab, float valeur){ 
    for (int i = size_tab-1; i>=1;i--){
        tab[i] = tab[i-1];
    }
    tab[0] = valeur;
}



void mpu9265_task(void *pvParameter) {
    // Wake up MPU9265
    i2c_write_byte(I2C_MASTER_0_PORT, MPU9265_ADDR, MPU9265_PWR_MGMT_1_REG, 0x00);
    
    calibrate_gyro();

    unsigned long lastTime = 0;
    int debug_counter = 0;
    while (1) { //loop()
        unsigned long now = esp_timer_get_time(); // microsecondes
        float dt = (now - lastTime) / 1000000.0f; // s
        lastTime = now;
        float sampleFreqLocal = 1.0f / dt;
        
        // Lecture Accéléromètre | faire une fonction qui renvoie un Vector3f ?
        Vector3f accel_vect = read_accel();

        fill_queue(tab_acce[0], SIZE_TAB, accel_vect.x);
        fill_queue(tab_acce[1], SIZE_TAB, accel_vect.y);
        fill_queue(tab_acce[2], SIZE_TAB, accel_vect.z);
        //mean_tab(tab_acce_x, SIZE_TAB);

        Vector3f gyro_vect = read_gyro();

        fill_queue(tab_gyro[0], SIZE_TAB, gyro_vect.x);
        fill_queue(tab_gyro[1], SIZE_TAB, gyro_vect.y);
        fill_queue(tab_gyro[2], SIZE_TAB, gyro_vect.z);

        // Lecture Magnétomètre | faire une fonction qui renvoie un Vector3f ?

        Vector3f magne_vect = read_magne();

        fill_queue(tab_magne[0], SIZE_TAB, magne_vect.x);
        fill_queue(tab_magne[1], SIZE_TAB, magne_vect.y);
        fill_queue(tab_magne[2], SIZE_TAB, magne_vect.z);
        // Conversion en Micro tesla possible, voir doc
        debug_counter += 1;
        if (debug_counter == 40){
            debug_counter = 0;
            //printf("Accel[g]: X=%.2f Y=%.2f Z=%.2f | Gyro[dps]: X=%.2f Y=%.2f Z=%.2f\n", accel_vect.x, accel_vect.y, accel_vect.z, gyro_vect.x, gyro_vect.y, gyro_vect.z);
            //ESP_LOGI(TAG, "XAccel[g]: X=%.2f Y=%.2f Z=%.2f | Gyro[dps]: X=%.2f Y=%.2f Z=%.2f", mean_tab(tab_acce[0], SIZE_TAB), mean_tab(tab_acce[1], SIZE_TAB), mean_tab(tab_acce[2], SIZE_TAB), mean_tab(tab_gyro[0], SIZE_TAB), mean_tab(tab_gyro[1], SIZE_TAB), mean_tab(tab_gyro[2], SIZE_TAB));
            //ESP_LOGI(TAG, "Magne: X=%.2f Y=%.2f Z=%.2f",magne_norm.x, magne_norm.y, magne_norm.z);
        }
        
        MadgwickAHRSupdate(sampleFreqLocal, mean_tab(tab_gyro[0], SIZE_TAB), mean_tab(tab_gyro[1], SIZE_TAB), mean_tab(tab_gyro[2], SIZE_TAB), mean_tab(tab_acce[0], SIZE_TAB), mean_tab(tab_acce[1], SIZE_TAB), mean_tab(tab_acce[2], SIZE_TAB), mean_tab(tab_magne[0], SIZE_TAB), mean_tab(tab_magne[1], SIZE_TAB), mean_tab(tab_magne[2], SIZE_TAB));
        // --- get Euler angles ---
        printf("%f,%f,%f,%f\n", q0, q1, q2, q3); // Envoi la donnée 
        vTaskDelay(pdMS_TO_TICKS((int)(0.02*1000)));
    }
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE); // désac les ESP LOGI
    i2c_master_init(I2C_NUM_0, I2C_MASTER_0_SDA_IO, I2C_MASTER_0_SCL_IO, I2C_MASTER_0_FREQ_HZ);
    i2c_master_init(I2C_NUM_1, I2C_MASTER_1_SDA_IO, I2C_MASTER_1_SCL_IO, I2C_MASTER_1_FREQ_HZ);
    qmc5883l_init();
    xTaskCreate(mpu9265_task, "mpu9265_task", 4096, NULL, 5, NULL);
}
