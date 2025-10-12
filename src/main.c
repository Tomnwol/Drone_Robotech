
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "i2c_basics.h"
#include "MadgwickAHRS.h"
#include "esp_timer.h"
#include "imu.h"

static const char *TAG = "MPU9265";

typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

Vector3f gyro_offset = {0};

#define SIZE_TAB  20
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

void calibrate_gyro() {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    const int samples = 200;  // nombre de mesures pour la moyenne

    uint8_t raw[6];
    for (int i = 0; i < samples; i++) {
        i2c_read_bytes(I2C_MASTER_0_PORT, MPU9265_ADDR, MPU9265_GYRO_XOUT_H, raw, 6);
        int16_t gx = (int16_t)((raw[0] << 8) | raw[1]);
        int16_t gy = (int16_t)((raw[2] << 8) | raw[3]);
        int16_t gz = (int16_t)((raw[4] << 8) | raw[5]);

        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        //ESP_LOGI(TAG, "Calibration gyro -> sum: X=%ld Y=%ld Z=%ld", sum_x, sum_y, sum_z);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    gyro_offset.x = (float)sum_x / samples;
    gyro_offset.y = (float)sum_y / samples;
    gyro_offset.z = (float)sum_z / samples;

    //ESP_LOGI(TAG, "Calibration gyro -> offset: X=%.2f Y=%.2f Z=%.2f", gyro_offset_x, gyro_offset_y, gyro_offset_z);
}

void mpu9265_task(void *pvParameter) {
    // Wake up MPU9265
    i2c_write_byte(I2C_MASTER_0_PORT, MPU9265_ADDR, MPU9265_PWR_MGMT_1_REG, 0x00);
    uint8_t raw_gyro[6];
    uint8_t raw_accel[6];
    uint8_t raw_magne[6];
    calibrate_gyro();

    unsigned long lastTime = 0;
    int debug_counter = 0;
    while (1) { //loop()
        unsigned long now = esp_timer_get_time(); // microsecondes
        float dt = (now - lastTime) / 1000000.0f; // s
        lastTime = now;
        float sampleFreqLocal = 1.0f / dt;
            // Lecture Accéléromètre
        
        i2c_read_bytes(I2C_MASTER_0_PORT, MPU9265_ADDR, MPU9265_ACCEL_XOUT_H, raw_accel, 6);
        int16_t accel_x = (raw_accel[0] << 8) | raw_accel[1];
        int16_t accel_y = (raw_accel[2] << 8) | raw_accel[3];
        int16_t accel_z = (raw_accel[4] << 8) | raw_accel[5];

        // Conversion en g (si plage = ±2g → 16384 LSB/g) ?
        Vector3f accel_vect = { (float)accel_x, (float)accel_y, (float)accel_z };
        fill_queue(tab_acce[0], SIZE_TAB, accel_vect.x);
        fill_queue(tab_acce[1], SIZE_TAB, accel_vect.y);
        fill_queue(tab_acce[2], SIZE_TAB, accel_vect.z);
        //mean_tab(tab_acce_x, SIZE_TAB);

        i2c_read_bytes(I2C_MASTER_0_PORT, MPU9265_ADDR, MPU9265_GYRO_XOUT_H, raw_gyro, 6);
        
        int16_t gyro_x = (raw_gyro[0] << 8) | raw_gyro[1];
        int16_t gyro_y = (raw_gyro[2] << 8) | raw_gyro[3];
        int16_t gyro_z = (raw_gyro[4] << 8) | raw_gyro[5];
          // Conversion en °/s. puis en rad/s (si plage = ±250°/s → 131 LSB/°/s)
        float gx = ((gyro_x - gyro_offset.x) / 131.0f) * (M_PI / 180.0f);
        float gy = ((gyro_y - gyro_offset.y) / 131.0f) * (M_PI / 180.0f);
        float gz = ((gyro_z - gyro_offset.z) / 131.0f) * (M_PI / 180.0f);

        fill_queue(tab_gyro[0], SIZE_TAB, gx);
        fill_queue(tab_gyro[1], SIZE_TAB, gy);
        fill_queue(tab_gyro[2], SIZE_TAB, gz);

        i2c_read_bytes(I2C_MASTER_1_PORT, QMC5883L_ADDR, QMC5883L_DATA_X_LSB, raw_magne, 6);
        
        int16_t magne_x = (raw_magne[1] << 8) | raw_magne[0];
        int16_t magne_y = (raw_magne[3] << 8) | raw_magne[2];
        int16_t magne_z = (raw_magne[5] << 8) | raw_magne[4];
          // Conversion en Micro tesla possible, voir doc
        Vector3f magne_vect = { (float)magne_x, (float)magne_y, (float)magne_z };
        debug_counter += 1;
        if (debug_counter == 40){
            debug_counter = 0;
            //ESP_LOGI(TAG, "Accel[g]: X=%.2f Y=%.2f Z=%.2f | Gyro[dps]: X=%.2f Y=%.2f Z=%.2f", acce_norm.x, acce_norm.y, acce_norm.z, gx, gy, gz);
            //ESP_LOGI(TAG, "XAccel[g]: X=%.2f Y=%.2f Z=%.2f | Gyro[dps]: X=%.2f Y=%.2f Z=%.2f", mean_tab(tab_acce[0], SIZE_TAB), mean_tab(tab_acce[1], SIZE_TAB), mean_tab(tab_acce[2], SIZE_TAB), mean_tab(tab_gyro[0], SIZE_TAB), mean_tab(tab_gyro[1], SIZE_TAB), mean_tab(tab_gyro[2], SIZE_TAB));
            //ESP_LOGI(TAG, "Magne: X=%.2f Y=%.2f Z=%.2f",magne_norm.x, magne_norm.y, magne_norm.z);
        }
        
        MadgwickAHRSupdate(sampleFreqLocal, gx, gy, gz, accel_vect.x, accel_vect.y, accel_vect.z, magne_vect.x, magne_vect.y, magne_vect.z);
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
