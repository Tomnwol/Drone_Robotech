#include <QGroupBox>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QString>
#include <QTimer>
#include <QCoreApplication>
#include "qtControllerBox.hpp"
#include "configuration.hpp"
#include "qtBoxConfiguration.hpp"
#include "UDP.hpp"
#include "controller.hpp"
#include "wifiConnection.hpp"
#include "qtStyles.hpp"

QGroupBox *startGroupBox = nullptr;
QTimer* timerUDPActivation = nullptr;

QCheckBox *connectToWiFiCheck = nullptr;
QCheckBox *UDPCheck = nullptr;

QTimer *timerWiFiVerification = nullptr;

void checkWiFiConnection(){
    if (getCurrentWifi() == ESP_SSID){
        connectToWiFiCheck->setText("WiFi Connected");
        QSignalBlocker blocker(connectToWiFiCheck);
        connectToWiFiCheck->setChecked(true);
        connectToWiFiCheck->setEnabled(false);
        UDPCheck->setEnabled(true);
    }else{
        connectToWiFiCheck->setText("WiFi Not Connected");
        QSignalBlocker blocker(connectToWiFiCheck);
        connectToWiFiCheck->setChecked(false);
        connectToWiFiCheck->setEnabled(true);
        UDPCheck->setChecked(false);
        UDPCheck->setEnabled(false);
    }
}

void initStartBox(QSerialPort* serial){
    /***2.Communication***/
    QGroupBox *communicationGroupBox = new QGroupBox("Communication");
    communicationGroupBox->setStyleSheet(NORMAL_LABEL_STYLE);
    QVBoxLayout *communicationVbox = new QVBoxLayout;

    QString connectToWiFi = QString("WiFi Connection");
    connectToWiFiCheck = new QCheckBox("WiFi Not Connected");
    connectToWiFiCheck->setStyleSheet(
        CLASSIC_CHECKBOX_STYLE
    );

    QCheckBox *controllerFoundCheck = new QCheckBox("Controller Not Found");
    controllerFoundCheck->setEnabled(false);
    controllerFoundCheck->setStyleSheet(
        CLASSIC_CHECKBOX_STYLE
    );
    QString UDPBoxName = QString("UDP communication (XBOX)");
    UDPCheck = new QCheckBox(UDPBoxName);
    UDPCheck->setStyleSheet(
        CLASSIC_CHECKBOX_STYLE
    );
    UDPCheck->setEnabled(false);
    //communicationVbox->addWidget(connectToWiFi);
    communicationVbox->addWidget(connectToWiFiCheck);
    communicationVbox->addWidget(UDPCheck);
    communicationVbox->addWidget(controllerFoundCheck);
    communicationGroupBox->setLayout(communicationVbox);

    /**1.Start**/
    startGroupBox = new QGroupBox("Start");
    startGroupBox->setStyleSheet(TITLE_LABEL_STYLE);
    QVBoxLayout *startVbox = new QVBoxLayout;
    startVbox->addWidget(communicationGroupBox);
    startGroupBox->setLayout(startVbox);
    checkWiFiConnection();

    timerWiFiVerification = new QTimer();
    timerWiFiVerification->setInterval(1000);
    QObject::connect(timerWiFiVerification, &QTimer::timeout, []() {
        checkWiFiConnection();
    });
    timerWiFiVerification->start();

    QObject::connect(connectToWiFiCheck, &QCheckBox::toggled, [&](bool checked){ // Active la manette et lance la communcation série
        connectToWiFiCheck->setText("WiFi Loading");
        connectToWiFiCheck->setStyleSheet(
            LOADING_LABEL_STYLE
        );
        QCoreApplication::processEvents();
        QSignalBlocker blocker(connectToWiFiCheck);
        checked = checked; // no effect, avoid warning
        bool isConnected = connectWifi(ESP_SSID);
        if (isConnected){
            connectToWiFiCheck->setChecked(true);
            UDPCheck->setEnabled(true);
            connectToWiFiCheck->setText("WiFi Connected");
        }else{
            connectToWiFiCheck->setChecked(false);
            connectToWiFiCheck->setText("WiFi Not Connected");
        }
        connectToWiFiCheck->setStyleSheet(CLASSIC_CHECKBOX_STYLE);
    });


    QObject::connect(UDPCheck, &QCheckBox::toggled, [&, serial](bool checked){ // Active la manette et lance la communcation série
        checked = checked; // no effect, avoid warning
        controllerGroupBox->setEnabled(true);
        KS_enable = true;
        noFocusPID();
        UDPCheck->setEnabled(false);
        configure_UDP();
    });

    timerUDPActivation = new QTimer();
    timerUDPActivation->setInterval(200); // 20 ms → 50 Hz

    QObject::connect(timerUDPActivation, &QTimer::timeout, [controllerFoundCheck]() {
        if( ButtonXBOX ){
            UDPCheck->setChecked(true);
        }
        controllerFoundCheck->setChecked(isControllerFound);
        if (isControllerFound){
            controllerFoundCheck->setText("Controller Found");
        }else{
            controllerFoundCheck->setText("Controller Not Found");
        }

    });
    timerUDPActivation->start();

}
