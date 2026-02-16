#ifndef __UDP__H__
#define __UDP__H__
#include <QUdpSocket>
#include <QTimer>
#include <QByteArray>
#include <QDebug>
#include <cstdint>
#include <QSerialPortInfo>
#include <iostream>
#define INITIAL_VALUES_PATH "initialValues.ini"
// Adresse IP et port de l'ESP32
const QString ESP32_IP = "192.168.4.1";  // IP ESP32 en mode AP
const quint16 ESP32_PORT = 4210;
#define PID_MULTIPLICATOR 100 // On mutiplie KP, KI et KD par 100 pour garder 2 décimales suplémentaires, il faudra /100 du côté drone par contre
typedef struct pl{
    uint16_t throttle = 0;
    uint16_t yaw = 0;
    uint16_t roll = 0;
    uint16_t pitch = 0;
    uint16_t GP_Pot = 0;
    uint8_t failSafeSwitch = 0;
    uint8_t killSwitch = 0;
    uint16_t KP = 0;
    uint16_t KI = 0;
    uint16_t KD = 0;
    uint8_t offsetMotorFL = 0;
    uint8_t offsetMotorFR = 0;
    uint8_t offsetMotorBL = 0;
    uint8_t offsetMotorBR = 0;
}Payload;
// QString initialValuesPath = "initialValues.ini";
extern Payload payload;
extern uint8_t droneBattery;
void configure_UDP();


#endif
