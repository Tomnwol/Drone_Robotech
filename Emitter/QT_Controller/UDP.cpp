#include "UDP.hpp"

QUdpSocket* udpSocket = nullptr;
QTimer* timer = nullptr;
uint8_t droneBattery= 255;
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
    udpSocket->bind(QHostAddress::AnyIPv4, 4211);
    timer = new QTimer();
    timer->setInterval(20); // 20 ms → 50 Hz

    QObject::connect(timer, &QTimer::timeout, []() {
        QByteArray frame;
        frame.resize(22); // Taille exacte de ton payload

        // Construction du payload (LSB puis MSB comme NRF)
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

        // Envoi UDP vers l'ESP32
        qint64 sent = udpSocket->writeDatagram(frame, QHostAddress(ESP32_IP), ESP32_PORT);
        if (sent != frame.size()) {
            std::cout << "Erreur envoi UDP !\n" ;
        }
    });
    timer->start();
    std::cout << "La communication UDP commence !\n" ;


    // Réception
    QObject::connect(udpSocket, &QUdpSocket::readyRead, [](){

        while (udpSocket->hasPendingDatagrams()) {

            QByteArray datagram;
            datagram.resize(udpSocket->pendingDatagramSize());

            QHostAddress sender;
            quint16 senderPort;

            udpSocket->readDatagram(datagram.data(),
                                    datagram.size(),
                                    &sender,
                                    &senderPort);

            if (datagram.size() == 15) { // Rajouter la récupération des autres paquets (pour CSV)
                droneBattery = datagram[0];
                droneYaw =     datagram[1] | (datagram[2] << 8);
                droneRoll =    datagram[3] | (datagram[4] << 8);
                dronePitch =   datagram[5] | (datagram[6] << 8);
                droneMotorFL = datagram[7] | (datagram[8] << 8);
                droneMotorFR = datagram[9] | (datagram[10] << 8);
                droneMotorBR = datagram[11] | (datagram[12] << 8);
                droneMotorBL = datagram[13] | (datagram[14] << 8);
            }
        }
    });

}

