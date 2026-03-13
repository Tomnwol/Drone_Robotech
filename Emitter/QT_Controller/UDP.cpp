#include <QDateTime>
#include <iostream>
#include "UDP.hpp"

QUdpSocket* udpSocket = nullptr;
QTimer* timer = nullptr;

uint8_t droneBattery = 255;
int16_t droneYaw = 0;
int16_t droneRoll = 0;
int16_t dronePitch = 0;
uint16_t droneMotorFL = 48;
uint16_t droneMotorFR = 48;
uint16_t droneMotorBR = 48;
uint16_t droneMotorBL = 48;

Payload payload;

void configure_UDP(){

    udpSocket = new QUdpSocket();

    // Bind propre
    udpSocket->bind(QHostAddress::AnyIPv4,
                    4211,
                    QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);

    timer = new QTimer();
    timer->setTimerType(Qt::PreciseTimer);
    timer->setInterval(20); // 50Hz

    // Buffer réutilisé pour éviter allocations
    static QByteArray frame(22, 0);

    QObject::connect(timer, &QTimer::timeout, [=]() {

        if(!udpSocket) return;

        // Construction du payload
        frame[0]  = payload.throttle & 0xFF;
        frame[1]  = (payload.throttle >> 8) & 0xFF;

        frame[2]  = payload.yaw & 0xFF;
        frame[3]  = (payload.yaw >> 8) & 0xFF;

        frame[4]  = payload.roll & 0xFF;
        frame[5]  = (payload.roll >> 8) & 0xFF;

        frame[6]  = payload.pitch & 0xFF;
        frame[7]  = (payload.pitch >> 8) & 0xFF;

        frame[8]  = payload.GP_Pot & 0xFF;
        frame[9]  = (payload.GP_Pot >> 8) & 0xFF;

        frame[10] = payload.failSafeSwitch;
        frame[11] = payload.killSwitch;

        frame[12] = payload.KP & 0xFF;
        frame[13] = (payload.KP >> 8) & 0xFF;

        frame[14] = payload.KI & 0xFF;
        frame[15] = (payload.KI >> 8) & 0xFF;

        frame[16] = payload.KD & 0xFF;
        frame[17] = (payload.KD >> 8) & 0xFF;

        frame[18] = payload.offsetMotorFL;
        frame[19] = payload.offsetMotorFR;
        frame[20] = payload.offsetMotorBL;
        frame[21] = payload.offsetMotorBR;

        qint64 sent = udpSocket->writeDatagram(
            frame,
            QHostAddress(ESP32_IP),
                                               ESP32_PORT);

        if (sent != frame.size()) {
            std::cout << "Erreur envoi UDP !" << std::endl;
        }
        qint64 timestamp = QDateTime::currentMSecsSinceEpoch();
        std::cout << "[" << timestamp << "] envoi UDP !" << std::endl;

    });

    timer->start();

    std::cout << "La communication UDP commence !" << std::endl;

    // Réception télémétrie
    QObject::connect(udpSocket, &QUdpSocket::readyRead, [=](){

        while (udpSocket->hasPendingDatagrams()) {

            QByteArray datagram;
            datagram.resize(udpSocket->pendingDatagramSize());

            QHostAddress sender;
            quint16 senderPort;

            udpSocket->readDatagram(datagram.data(),
                                    datagram.size(),
                                    &sender,
                                    &senderPort);

            if (datagram.size() == 15) {

                const uint8_t* d = reinterpret_cast<const uint8_t*>(datagram.constData());

                droneBattery = d[0];

                droneYaw   = (int16_t)((uint16_t)d[1]  | ((uint16_t)d[2]  << 8));
                droneRoll  = (int16_t)((uint16_t)d[3]  | ((uint16_t)d[4]  << 8));
                dronePitch = (int16_t)((uint16_t)d[5]  | ((uint16_t)d[6]  << 8));

                droneMotorFL = (uint16_t)d[7]  | ((uint16_t)d[8]  << 8);
                droneMotorFR = (uint16_t)d[9]  | ((uint16_t)d[10] << 8);
                droneMotorBR = (uint16_t)d[11] | ((uint16_t)d[12] << 8);
                droneMotorBL = (uint16_t)d[13] | ((uint16_t)d[14] << 8);
            }
        }
    });

}
