#ifndef __I2C_BASICS
#define __I2C_BASICS


#define I2C_MASTER_0_SCL_IO           22
#define I2C_MASTER_0_SDA_IO           21
#define I2C_MASTER_0_FREQ_HZ          400000
#define I2C_MASTER_0_PORT             I2C_NUM_0

#define I2C_MASTER_1_SCL_IO           25
#define I2C_MASTER_1_SDA_IO           26
#define I2C_MASTER_1_FREQ_HZ          400000
#define I2C_MASTER_1_PORT             I2C_NUM_1

esp_err_t i2c_write_byte(int i2c_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

esp_err_t i2c_read_bytes(int i2c_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

void i2c_master_init(int i2c_num, int sda_io, int scl_io, uint32_t clk_speed);

#endif
