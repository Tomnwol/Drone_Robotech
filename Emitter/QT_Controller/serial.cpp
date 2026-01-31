#include <QSerialPort>
#include <QSerialPortInfo>

#include "serial.hpp"

#define PORT_PATH "/dev/ttyUSB0"
#define UART_BAUDRATE QSerialPort::Baud115200

void configure_serial(QSerialPort* serial){
    // serial.setPortName("COM3");              // Windows
    serial->setPortName(PORT_PATH);   // Linux
    serial->setBaudRate(UART_BAUDRATE);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    /*if (!serial->open(QIODevice::ReadWrite)) {
        std::cout << "Failed to open serial port\n";
        return;
    }*/
    QTimer *uartTimer = new QTimer(serial);
    uartTimer->setInterval(10);

    QObject::connect(uartTimer, &QTimer::timeout,
                     [serial]() {
                         QByteArray frame;
                         frame.append(payload.throttle >> 8); //Throttle
                         frame.append(payload.throttle && 255);
                         frame.append(payload.killSwitch); //KillSwitch
                         frame.append(payload.failSafeSwitch); //FailSafe
                         frame.append(payload.KP); //KP
                         frame.append(payload.KI); //KI
                         std::cout << "throttle: " << static_cast<int>(payload.throttle) << "\n";
                         //serial->write(frame);
                         //std::cout << "UART frame sent\n";
                     });

    uartTimer->start();
}
