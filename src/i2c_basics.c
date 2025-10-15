#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c_basics.h"
#include "driver/i2c.h"
#include "types.h"
esp_err_t i2c_write_byte(int i2c_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(i2c_num, dev_addr, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

esp_err_t i2c_read_bytes(int i2c_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(i2c_num, dev_addr, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

void i2c_master_init(int i2c_num, int sda_io, int scl_io, uint32_t clk_speed) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_speed,
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, conf.mode, 0, 0, 0));
}



