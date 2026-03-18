#include <QGroupBox>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QString>
#include <QTimer>
#include <QCoreApplication>
#include <QtConcurrent/QtConcurrent>
#include <QFutureWatcher>
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
QFutureWatcher<QString> *wifiWatcher = nullptr;
bool wifiCheckInProgress = false;
/*void checkWiFiConnection(){
    std::cout << "CHECK CONNECTION BEGIN" << std::endl;
    if (getCurrentWifi() == ESP_SSID){
        connectToWiFiCheck->setText("WiFi Connected");
        QSignalBlocker blocker(connectToWiFiCheck);
        connectToWiFiCheck->setChecked(true);
        connectToWiFiCheck->setEnabled(false);
        //UDPCheck->setEnabled(true);
    }else{
        connectToWiFiCheck->setText("WiFi Not Connected (MENU)");
        QSignalBlocker blocker(connectToWiFiCheck);
        connectToWiFiCheck->setChecked(false);
        connectToWiFiCheck->setEnabled(true);
        UDPCheck->setChecked(false);
        UDPCheck->setEnabled(false);
    }
    std::cout << "CHECK CONNECTION END" << std::endl;
}*/

// Appelée dans le thread principal — met à jour l'UI
void onWifiCheckFinished() {
    QString ssid = wifiWatcher->result();
    wifiCheckInProgress = false;

    if (ssid == ESP_SSID) {
        connectToWiFiCheck->setText("WiFi Connected");
        QSignalBlocker blocker(connectToWiFiCheck);
        connectToWiFiCheck->setChecked(true);
        connectToWiFiCheck->setEnabled(false);
    } else {
        connectToWiFiCheck->setText("WiFi Not Connected (MENU)");
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
    //communicationGroupBox->setStyleSheet(NORMAL_LABEL_STYLE);
    QVBoxLayout *communicationVbox = new QVBoxLayout;

    QString connectToWiFi = QString("WiFi Connection");
    connectToWiFiCheck = new QCheckBox("WiFi Not Connected (MENU)");
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

    wifiWatcher = new QFutureWatcher<QString>(nullptr);  // ← plus de this
    QObject::connect(wifiWatcher, &QFutureWatcher<QString>::finished,
                     []() { onWifiCheckFinished(); });    // ← plus de this

    timerWiFiVerification = new QTimer(nullptr);          // ← plus de this
    QObject::connect(timerWiFiVerification, &QTimer::timeout, []() {  // ← plus de this
        if (!wifiCheckInProgress) {
            wifiCheckInProgress = true;
            wifiWatcher->setFuture(QtConcurrent::run(getCurrentWifi));
        }
    });

    timerWiFiVerification->start();
    wifiCheckInProgress = true;
    wifiWatcher->setFuture(QtConcurrent::run(getCurrentWifi));
    //checkWiFiConnection();
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
            connectToWiFiCheck->setText("WiFi Not Connected (MENU)");
        }
        connectToWiFiCheck->setStyleSheet(CLASSIC_CHECKBOX_STYLE);
    });


    QObject::connect(UDPCheck, &QCheckBox::toggled, [&, serial](bool checked){ // Active la manette et lance la communcation série
        if (checked){
            controllerGroupBox->setEnabled(true);
            KS_enable = true;
            noFocusPID();
            UDPCheck->setEnabled(false);
            configure_UDP();
        }
    });

    timerUDPActivation = new QTimer();
    timerUDPActivation->setInterval(200); // 20 ms → 50 Hz

    QObject::connect(timerUDPActivation, &QTimer::timeout, [controllerFoundCheck]() {
        if( ButtonMENU ){
            connectToWiFiCheck->setChecked(true);
        }

        if( ButtonXBOX && connectToWiFiCheck->isChecked()){
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
