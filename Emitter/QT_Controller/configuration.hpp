#ifndef __CONFIGURATION__H__
#define __CONFIGURATION__H__
#include <QSettings>
struct Config {
    float Kp;
    float Ki;
    float Kd;
    uint8_t offsetMotorFL;
    uint8_t offsetMotorFR;
    uint8_t offsetMotorBL;
    uint8_t offsetMotorBR;
};

extern Config my_config;
Config loadConfig(const QString &filename);
void saveConfig(const QString &filename);
#endif
