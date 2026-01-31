#include <QApplication>
#include <QWidget>
#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QCheckBox>
#include <QShortcut>
#include <QDoubleSpinBox>
#include <QFont>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <iostream>

#include "serial.hpp"

#define DSHOT_MIN 48
#define DSHOT_MAX 2047

#define K_p 1
#define K_i 1

QSerialPort* serial = new QSerialPort();

int main(int argc, char *argv[])
{
    /*APPLICATION SETUP*/
    QApplication app(argc, argv);
    QWidget window;
    window.setWindowTitle("Drone Controller Interface");
    QVBoxLayout layout;

    /***2.Communication***/
    QGroupBox *communicationGroupBox = new QGroupBox("Communication");
    QVBoxLayout *communicationVbox = new QVBoxLayout;
    QString serialBoxName = QString("Serial (%1)").arg(UART_BAUDRATE_STR);
    QCheckBox *serialCheck = new QCheckBox(serialBoxName);
    communicationVbox->addWidget(serialCheck);
    communicationGroupBox->setLayout(communicationVbox);

    /***2.Security Box***/
    QGroupBox *securityGroupBox = new QGroupBox("Security");
    QCheckBox *FS_Check = new QCheckBox("FailSafe Switch (toggle on F)");
    QCheckBox *killCheck = new QCheckBox("Kill Switch (space bar)");
    QVBoxLayout *securityVbox = new QVBoxLayout;
    securityVbox->addWidget(FS_Check);
    securityVbox->addWidget(killCheck);
    securityVbox->addStretch(1);
    securityGroupBox->setLayout(securityVbox);

    /***2.Motors Sliders***/
    QGroupBox *motorsGroupBox = new QGroupBox("Motors");
    QVBoxLayout *motorsVbox = new QVBoxLayout;
    QLabel *throttleLabel = new QLabel("");
    QSlider *throttleSlider = new QSlider(Qt::Horizontal);
    throttleSlider->setRange(DSHOT_MIN, DSHOT_MAX);
    throttleSlider->setValue(DSHOT_MIN);
    throttleLabel->setText("Throttle Value : " + QString::number(throttleSlider->value()));
    motorsVbox->addWidget(throttleLabel);
    motorsVbox->addWidget(throttleSlider);
    motorsGroupBox->setLayout(motorsVbox);

    /***2.PI SpinBox***/
    QGroupBox *PI_GroupBox = new QGroupBox("PI Values");
    QVBoxLayout *vbox_PI = new QVBoxLayout;
    QLabel *kpLabel = new QLabel("Kp:");
    QDoubleSpinBox *kpSpin = new QDoubleSpinBox();
    kpSpin->setRange(0.0, 100.0);    // plage de valeurs
    kpSpin->setSingleStep(0.1);      // incrément à chaque flèche
    kpSpin->setValue(1.0);           // valeur par défaut
    kpSpin->setDecimals(2);          // nombre de décimales affichées

    QLabel *kiLabel = new QLabel("Ki:");
    QDoubleSpinBox *kiSpin = new QDoubleSpinBox();
    kiSpin->setRange(0.0, 100.0);    // plage de valeurs
    kiSpin->setSingleStep(0.1);      // incrément à chaque flèche
    kiSpin->setValue(1.0);           // valeur par défaut
    kiSpin->setDecimals(2);          // nombre de décimales affichées

    vbox_PI->addWidget(kpLabel);
    vbox_PI->addWidget(kpSpin);
    vbox_PI->addWidget(kiLabel);
    vbox_PI->addWidget(kiSpin);
    PI_GroupBox->setLayout(vbox_PI);

    /**1.Start**/
    QGroupBox *startGroupBox = new QGroupBox("Start");
    QVBoxLayout *startVbox = new QVBoxLayout;
    startVbox->addWidget(communicationGroupBox);
    startGroupBox->setLayout(startVbox);

    /**1.Configuration**/
    QGroupBox *configurationGroupBox = new QGroupBox("Configuration");
    QVBoxLayout *configurationVbox = new QVBoxLayout;
    configurationVbox->addWidget(PI_GroupBox);
    configurationGroupBox->setLayout(configurationVbox);

    /**1.Controller**/
    QGroupBox *controllerGroupBox = new QGroupBox("Controller");
    controllerGroupBox->setEnabled(false); // Control non disponible tant que le serial n'est pas activé
    QVBoxLayout *controllerVbox = new QVBoxLayout;
    controllerVbox->addWidget(motorsGroupBox);
    controllerVbox->addWidget(securityGroupBox);
    controllerGroupBox->setLayout(controllerVbox);

    /*0.Main Box*/
    QGroupBox *mainGroupBox = new QGroupBox("");
    QHBoxLayout *mainHBox = new QHBoxLayout;
    mainHBox->addWidget(startGroupBox);
    mainHBox->addWidget(configurationGroupBox);
    mainHBox->addWidget(controllerGroupBox);
    mainGroupBox->setLayout(mainHBox);

    /*Display*/
    layout.addWidget(mainGroupBox);
    window.setLayout(&layout);
    window.show();

    /*Connections*/
    QShortcut *FS_Shortcut = new QShortcut(QKeySequence(Qt::Key_F), &window);
    QObject::connect(FS_Shortcut, &QShortcut::activated, [&]() { //Active/Désactive la limitation moteur (côté drone)
        FS_Check->setChecked(not FS_Check->isChecked());
    });

    QShortcut *killShortcut = new QShortcut(QKeySequence(Qt::Key_Space), &window);
    QObject::connect(killShortcut, &QShortcut::activated, [&]() { //Désactive le drone
        killCheck->setChecked(true);
    });

    QObject::connect(throttleSlider, &QSlider::valueChanged, [throttleLabel](int value){ //Update la valeur de Throttle
        throttleLabel->setText("Throttle Value : " + QString::number(value));
    });

    QObject::connect(serialCheck, &QCheckBox::toggled, [controllerGroupBox,serialCheck](bool checked){
        controllerGroupBox->setEnabled(checked); // Active la manette et lance la communcation série
        serialCheck->setEnabled(false);
        configure_serial(serial);
    });



    return app.exec();
}
