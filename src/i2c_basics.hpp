#ifndef __I2C_BASICS
#define __I2C_BASICS

#include <Wire.h>
#include <Arduino.h>
#include "driver/i2c.h"

// Pins et fréquence I2C
extern TwoWire I2C_MASTER_0_PORT;       // Bus I2C matériel 0
extern TwoWire I2C_MASTER_1_PORT; // Bus I2C matériel 1

#define I2C_MASTER_0_SCL_IO 22
#define I2C_MASTER_0_SDA_IO 21
#define I2C_MASTER_0_FREQ_HZ 400000

#define I2C_MASTER_1_SCL_IO 25
#define I2C_MASTER_1_SDA_IO 26
#define I2C_MASTER_1_FREQ_HZ 400000

// Prototype des fonctions
bool i2c_write_byte(TwoWire &bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
bool i2c_read_bytes(TwoWire &bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len);
void i2c_master_init(TwoWire &bus, int sda_pin, int scl_pin, uint32_t clk_speed);

#endif
