#ifndef __SERIAL__H__
#define __SERIAL__H__
#include <QSerialPort>
#include <QTimer>
#include <QSerialPortInfo>
#include <iostream>
#define UART_BAUDRATE_STR "115200"
#define UART_BAUDRATE QSerialPort::Baud115200

#define PID_MULTIPLICATOR 100 // On mutiplie KP, KI et KD par 100 pour garder 2 décimales suplémentaires, il faudra /100 du côté drone par contre
typedef struct pl{
    uint16_t throttle = 0;
    uint8_t killSwitch = 0;
    uint8_t failSafeSwitch = 0;
    uint16_t KP = 0;
    uint16_t KI = 0;
    uint16_t KD = 0;
    uint8_t offsetMotorFL = 0;
    uint8_t offsetMotorFR = 0;
    uint8_t offsetMotorBL = 0;
    uint8_t offsetMotorBR = 0;
}Payload;

extern Payload payload;
void configure_serial(QSerialPort*);


#endif
