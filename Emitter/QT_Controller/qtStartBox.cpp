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
    communicationGroupBox->setStyleSheet("QGroupBox { font-weight: normal; }");
    QVBoxLayout *communicationVbox = new QVBoxLayout;
    QString UDPBoxName = QString("UDP (XBOX)");
    QCheckBox *controllerFoundCheck = new QCheckBox("Controller Not Found");
    controllerFoundCheck->setEnabled(false);
    controllerFoundCheck->setStyleSheet(
        "QCheckBox::indicator:unchecked {"
        "background-color: #ffdddd;"
        "border: 1px solid gray;"
        "}"
        "QCheckBox::indicator:checked {"
        "background-color: #00AA00;"
        "border: 1px solid black;"
        "}"
    );
    QCheckBox *UDPCheck = new QCheckBox(UDPBoxName);
    UDPCheck->setStyleSheet(
        "QCheckBox::indicator:unchecked {"
        "background-color: #ffdddd;"
        "border: 1px solid gray;"
        "}"
        "QCheckBox::indicator:checked {"
        "background-color: #00AA00;"
        "border: 1px solid black;"
        "}"
    );
    communicationVbox->addWidget(UDPCheck);
    communicationVbox->addWidget(controllerFoundCheck);
    communicationGroupBox->setLayout(communicationVbox);

    /**1.Start**/
    startGroupBox = new QGroupBox("Start");
    startGroupBox->setStyleSheet("QGroupBox { font-family: 'DejaVu Sans Mono'; font-size: 16px;  font-weight: bold; }");
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
    timerUDPActivation->setInterval(200); // 20 ms → 50 Hz

    QObject::connect(timerUDPActivation, &QTimer::timeout, [UDPCheck, controllerFoundCheck]() {
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
