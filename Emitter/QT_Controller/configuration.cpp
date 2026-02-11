#include "configuration.hpp"

Config loadConfig(const QString &filename) {
    Config my_config;
    QSettings settings(filename, QSettings::IniFormat);

    settings.beginGroup("PID");
    my_config.Kp = settings.value("Kp", 0.0).toFloat(); // 0.0 par défaut
    my_config.Ki = settings.value("Ki", 0.0).toFloat();
    my_config.Kd = settings.value("Kd", 0.0).toFloat();
    settings.endGroup();

    return my_config;
}

void saveConfig(const QString &filename, const Config &my_config) {
    QSettings settings(filename, QSettings::IniFormat);

    settings.beginGroup("PID");
    settings.setValue("Kp", my_config.Kp);
    settings.setValue("Ki", my_config.Ki);
    settings.setValue("Kd", my_config.Kd);
    settings.endGroup();
}
