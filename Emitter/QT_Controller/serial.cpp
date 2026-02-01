#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include "serial.hpp"

#define PORT_PATH "/dev/ttyACM0"
#define UART_BAUDRATE QSerialPort::Baud115200

void configure_serial(QSerialPort* serial){
    // serial.setPortName("COM3");              // Windows
    serial->setPortName(PORT_PATH);   // Linux
    serial->setBaudRate(UART_BAUDRATE);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    if (!serial->open(QIODevice::ReadWrite)) {
        std::cout << "Failed to open serial port\n";
        return;
    }else{
        std::cout << "Serial port opened\n";
    }
    //LECTURE (DEBUG)
    QObject::connect(serial, &QSerialPort::readyRead, [serial]() {
        static QByteArray rxBuffer;
        rxBuffer.append(serial->readAll());

        // Lecture ligne par ligne (ACK\n par exemple)
        while (rxBuffer.contains('\n')) {
            int idx = rxBuffer.indexOf('\n');
            QByteArray line = rxBuffer.left(idx);
            rxBuffer.remove(0, idx + 1);

            std::cout << "READ_ARDUINO:" << line.constData() << "\n";
        }
    });

    //ECRITURE

    QTimer *uartTimer = new QTimer(serial);
    uartTimer->setInterval(10);

    QObject::connect(uartTimer, &QTimer::timeout,
                     [serial]() {
                         QByteArray frame; //[LSB puis MSB]
                         frame.append(payload.throttle & 255); //Throttle
                         frame.append(payload.throttle >> 8);
                         frame.append(payload.killSwitch); //KillSwitch
                         frame.append(payload.failSafeSwitch); //FailSafe
                         frame.append(payload.KP & 255); //KP
                         frame.append(payload.KP >> 8);
                         frame.append(payload.KI & 255); //KI
                         frame.append(payload.KI >> 8);
                         frame.append('\n');
                         serial->write(frame);
                     });

    uartTimer->start();
}
