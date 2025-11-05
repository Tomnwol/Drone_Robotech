#include "types.h"
#include "driver/i2c.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c_basics.h"
#include "imu.h"


#define PI 3.14159265
Vector3f gyro_offset = {0};

void qmc5883l_init(void) { // Voir datasheet https://www.qstcorp.com/upload/pdf/202202/%EF%BC%88%E5%B7%B2%E4%BC%A0%EF%BC%8913-52-19%20QMC5883P%20Datasheet%20Rev.C(1).pdf
    // Reset
    i2c_write_byte(I2C_MASTER_1_PORT, QMC5883L_ADDR, QMC5883L_MODE_REG, 0xC3); //Continious mode
    vTaskDelay(pdMS_TO_TICKS(100));

    i2c_write_byte(I2C_MASTER_1_PORT, QMC5883L_ADDR, 0x0B, 0x08);
    vTaskDelay(pdMS_TO_TICKS(50));

    i2c_write_byte(I2C_MASTER_1_PORT, QMC5883L_ADDR, 0x29, 0x06);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Set/Reset
    i2c_write_byte(I2C_MASTER_1_PORT, QMC5883L_ADDR, QMC5883L_SET_RESET, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));
}

void calibrate_gyro() {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    const int samples = 400;  // nombre de mesures pour la moyenne

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

Vector3f read_accel() {
    uint8_t raw_accel[6];
    i2c_read_bytes(I2C_MASTER_0_PORT, MPU9265_ADDR, MPU9265_ACCEL_XOUT_H, raw_accel, 6);
    int16_t accel_x = (raw_accel[0] << 8) | raw_accel[1];
    int16_t accel_y = (raw_accel[2] << 8) | raw_accel[3];
    int16_t accel_z = (raw_accel[4] << 8) | raw_accel[5];
    return (Vector3f){ (float)accel_x, (float)accel_y, (float)accel_z };
}

Vector3f read_gyro() {
    uint8_t raw_gyro[6];
    i2c_read_bytes(I2C_MASTER_0_PORT, MPU9265_ADDR, MPU9265_GYRO_XOUT_H, raw_gyro, 6);
    int16_t gyro_x = (raw_gyro[0] << 8) | raw_gyro[1];
    int16_t gyro_y = (raw_gyro[2] << 8) | raw_gyro[3];
    int16_t gyro_z = (raw_gyro[4] << 8) | raw_gyro[5];
    float gx = ((gyro_x - gyro_offset.x) / 131.0f) * (PI / 180.0f);
    float gy = ((gyro_y - gyro_offset.y) / 131.0f) * (PI / 180.0f);
    float gz = ((gyro_z - gyro_offset.z) / 131.0f) * (PI / 180.0f);
    return (Vector3f){(float)gx, (float)gy, (float)gz};
}

Vector3f read_magne() {
        uint8_t raw_magne[6];
        i2c_read_bytes(I2C_MASTER_1_PORT, QMC5883L_ADDR, QMC5883L_DATA_X_LSB, raw_magne, 6);
        //int16_t magne_x = (raw_magne[3] << 8) | raw_magne[2];
        //int16_t magne_y = (raw_magne[5] << 8) | raw_magne[4];
        //int16_t magne_z = (raw_magne[1] << 8) | raw_magne[0];
        int16_t magne_x = (raw_magne[1] << 8) | raw_magne[0];
        int16_t magne_y = (raw_magne[3] << 8) | raw_magne[2];
        int16_t magne_z = (raw_magne[5] << 8) | raw_magne[4];

        float hard_iron_bias_x =   -14.092505307175724 ;
        float hard_iron_bias_y =   732.1141683831913 ;
        float hard_iron_bias_z =   -714.4415414077869 ;


        double soft_iron_bias_xx = 2.1729320914696246 ;
        double soft_iron_bias_xy =  0.08319794986580793 ;
        double soft_iron_bias_xz =  0.035064382876120764 ;


        double soft_iron_bias_yx =   0.08319794986580807 ;
        double soft_iron_bias_yy =   1.6228055535235297;
        double soft_iron_bias_yz =  0.4682315399054801 ;


        double soft_iron_bias_zx =  0.035064382876120855 ;
        double soft_iron_bias_zy =  0.4682315399054797 ;
        double soft_iron_bias_zz =  1.48724612402111 ;
        
        // get values x,y,z and subtract the hard iron offset
        float xm_off = (float)magne_x - hard_iron_bias_x;
        float ym_off = (float)magne_y - hard_iron_bias_y;
        float zm_off = (float)magne_z - hard_iron_bias_z;
        
        // multiply by the inverse soft iron offset 
        float xm_cal = xm_off *  soft_iron_bias_xx + ym_off *  soft_iron_bias_yx  + zm_off *  soft_iron_bias_zx;
        float ym_cal = xm_off *  soft_iron_bias_xy + ym_off *  soft_iron_bias_yy + zm_off *  soft_iron_bias_zy;
        float zm_cal = xm_off *  soft_iron_bias_xz + ym_off *  soft_iron_bias_yz  + zm_off *  soft_iron_bias_zz;

        //Hard Iron, valeurs trouvées expérimentalement servant à régler les problèmes de détection magnétique
        //Procédure pour obtenir ces valeurs | Axe Z doit être ~0 à plat. Pour l'axe X et Y, il faut repérer deux extremums pour essayer de voir si le capteur voir globalement trop de positif ou trop de négatif. Il faut garder en tête qu'une rotation de 180° autour de Z doit donner la valeur opposé sur X et Y. Exemple : x vaut 25 si on tourne le capteur de 180°, la valeur lue doit désormais être d'environ -25 
        //magne_x += 20;
        //magne_y -= 1025;
        //magne_z += 242;
        return     (Vector3f){ (float)0, (float)0, (float)0 };
        //return     (Vector3f){ (float)xm_cal, (float)ym_cal, (float)zm_cal };
}
