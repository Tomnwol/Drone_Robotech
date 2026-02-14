#include "configuration.hpp"

Config my_config;

Config loadConfig(const QString &filename) {
    //Config my_config;
    QSettings settings(filename, QSettings::IniFormat);

    settings.beginGroup("PID");
    my_config.Kp = settings.value("Kp", 0.0).toFloat(); // 0.0 par défaut
    my_config.Ki = settings.value("Ki", 0.0).toFloat();
    my_config.Kd = settings.value("Kd", 0.0).toFloat();
    settings.endGroup();
    settings.beginGroup("OFFSET");
    my_config.offsetMotorFL = settings.value("offsetMotorFL", 0.0).toInt();
    my_config.offsetMotorFR = settings.value("offsetMotorFR", 0.0).toInt();
    my_config.offsetMotorBL = settings.value("offsetMotorBL", 0.0).toInt();
    my_config.offsetMotorBR = settings.value("offsetMotorBR", 0.0).toInt();
    settings.endGroup();

    return my_config;
}

void saveConfig(const QString &filename) {
    QSettings settings(filename, QSettings::IniFormat);

    settings.beginGroup("PID");
    settings.setValue("Kp", my_config.Kp);
    settings.setValue("Ki", my_config.Ki);
    settings.setValue("Kd", my_config.Kd);
    settings.endGroup();
    settings.beginGroup("OFFSET");
    settings.setValue("offsetMotorFL", my_config.offsetMotorFL);
    settings.setValue("offsetMotorFR", my_config.offsetMotorFR);
    settings.setValue("offsetMotorBL", my_config.offsetMotorBL);
    settings.setValue("offsetMotorBR", my_config.offsetMotorBR);
    settings.endGroup();
}
