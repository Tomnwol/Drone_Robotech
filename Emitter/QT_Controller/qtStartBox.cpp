#include <QGroupBox>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QString>
#include <QTimer>
#include "qtControllerBox.hpp"
#include "configuration.hpp"
#include "qtBoxConfiguration.hpp"
#include "UDP.hpp"
#include "controller.hpp"

QGroupBox *startGroupBox = nullptr;
QTimer* timerUDPActivation = nullptr;
void initStartBox(QSerialPort* serial){
    /***2.Communication***/
    QGroupBox *communicationGroupBox = new QGroupBox("Communication");
    QVBoxLayout *communicationVbox = new QVBoxLayout;
    QString UDPBoxName = QString("UDP (%1)").arg(ESP32_PORT);
    QCheckBox *UDPCheck = new QCheckBox(UDPBoxName);
    communicationVbox->addWidget(UDPCheck);
    communicationGroupBox->setLayout(communicationVbox);

    /**1.Start**/
    startGroupBox = new QGroupBox("Start");
    QVBoxLayout *startVbox = new QVBoxLayout;
    startVbox->addWidget(communicationGroupBox);
    startGroupBox->setLayout(startVbox);

    QObject::connect(UDPCheck, &QCheckBox::toggled, [&, UDPCheck, serial](bool checked){ // Active la manette et lance la communcation série
        checked = checked; // no effect, avoid warning
        controllerGroupBox->setEnabled(true);
        KS_enable = true;
        noFocusPID();
        UDPCheck->setEnabled(false);
        configure_UDP();
    });

    timerUDPActivation = new QTimer();
    timerUDPActivation->setInterval(20); // 20 ms → 50 Hz

    QObject::connect(timerUDPActivation, &QTimer::timeout, [UDPCheck]() {
        if( ButtonXBOX ){
            UDPCheck->setChecked(true);
        }
        timerUDPActivation->stop();
    });
    timerUDPActivation->start();



}
