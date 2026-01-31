#ifndef __SERIAL__H__
#define __SERIAL__H__
#include <QSerialPort>
#include <QTimer>
#include <QSerialPortInfo>
#include <iostream>
#define UART_BAUDRATE_STR "115200"
#define UART_BAUDRATE QSerialPort::Baud115200

void configure_serial(QSerialPort*);

typedef struct pl{
    uint8_t KP = 0;
    uint8_t KI = 0;
    uint16_t throttle = 0;
    uint8_t killSwitch = 0;
    uint8_t failSafeSwitch = 0;
}Payload;
#endif
