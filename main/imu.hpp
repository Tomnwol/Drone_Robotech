#pragma once
#include <Arduino.h>
#include "types.hpp"

#define MPU9265_ADDR                0x68
#define MPU9265_WHO_AM_I_REG        0x75
#define MPU9265_PWR_MGMT_1_REG      0x6B
#define MPU9265_GYRO_XOUT_H         0x43
#define MPU9265_ACCEL_XOUT_H        0x3B

#define QMC5883L_ADDR       0x2C
#define QMC5883L_DATA_X_LSB 0x01
#define QMC5883L_DATA_X_MSB 0x02
#define QMC5883L_DATA_Y_LSB 0x03
#define QMC5883L_DATA_Y_MSB 0x04
#define QMC5883L_DATA_Z_LSB 0x05
#define QMC5883L_DATA_Z_MSB 0x06
#define QMC5883L_STATUS     0x09
#define QMC5883L_MODE_REG 0x0A
#define QMC5883L_SET_RESET 0x0B

void qmc5883l_init(void);
Vector3f read_accel();
Vector3f read_gyro();
Vector3f read_magne();
void calibrate_gyro();

bool isMagnePresent();
bool isMPUPresent();

Euler get_att_desired();
