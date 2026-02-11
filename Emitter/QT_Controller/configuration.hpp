#ifndef __CONFIGURATION__H__
#define __CONFIGURATION__H__
#include <QSettings>
struct Config {
    float Kp;
    float Ki;
    float Kd;
};

Config loadConfig(const QString &filename);
void saveConfig(const QString &filename, const Config &my_config);
#endif
