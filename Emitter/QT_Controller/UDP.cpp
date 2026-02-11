



void configure_UDP(){
    // --- Création de la socket UDP ---
    QUdpSocket udpSocket;

    // --- Timer pour envoi périodique ---
    QTimer timer;
    timer.setInterval(20); // 20 ms → 50 Hz

    QObject::connect(&timer, &QTimer::timeout, [&]() {
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
        frame[10] = payload.SWFailSafe;
        frame[11] = payload.SWKillSwitch;
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
        qint64 sent = udpSocket.writeDatagram(frame, QHostAddress(ESP32_IP), ESP32_PORT);
        if (sent != frame.size()) {
            qDebug() << "Erreur envoi UDP !";
        }
});

timer.start();
