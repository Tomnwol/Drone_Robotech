#include "imu.h"
#include "driver/i2c.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c_basics.h"

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

