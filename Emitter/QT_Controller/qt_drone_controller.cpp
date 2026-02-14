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
#include <QKeyEvent>
#include <QPushButton>
#include <iostream>

#include "UDP.hpp"
#include "configuration.hpp"
#include "qtBoxConfiguration.hpp"
#define OFFSET_MOTOR_MAX 100

#define DSHOT_MIN 48
#define DSHOT_MAX 2047


Payload payload;

class NoKeyboardSpinBox : public QDoubleSpinBox {
public:
    using QDoubleSpinBox::QDoubleSpinBox;

protected:
    void keyPressEvent(QKeyEvent *event) override {
        event->ignore(); // ignore TOUT clavier
    }
};

int main(int argc, char *argv[])
{

    /*APPLICATION SETUP*/
    QApplication app(argc, argv);
    QWidget window;
    QSerialPort* serial = new QSerialPort(&window);
    window.setWindowTitle("Drone Controller Interface");
    QVBoxLayout layout;
    my_config = loadConfig(INITIAL_VALUES_PATH);

    /****3.Left offset****/
    QLabel *OM_FL_Label = new QLabel("");
    QSlider *OM_FL_Slider = new QSlider(Qt::Horizontal);
    //OM_FL_Slider->setInvertedAppearance(true);
    OM_FL_Slider->setRange(0, OFFSET_MOTOR_MAX);
    OM_FL_Slider->setValue(my_config.offsetMotorFL);
    payload.offsetMotorFL = 0;
    OM_FL_Label->setText("OM_FL Value : " + QString::number(OM_FL_Slider->value()));

    QLabel *OM_BL_Label = new QLabel("");
    QSlider *OM_BL_Slider = new QSlider(Qt::Horizontal);
    OM_BL_Slider->setRange(0, OFFSET_MOTOR_MAX);
    OM_BL_Slider->setValue(my_config.offsetMotorBL);
    payload.offsetMotorBL = 0;
    OM_BL_Label->setText("OM_BL Value : " + QString::number(OM_BL_Slider->value()));

    /****3.Right offset****/
    QLabel *OM_FR_Label = new QLabel("");
    QSlider *OM_FR_Slider = new QSlider(Qt::Horizontal);
    OM_FR_Slider->setRange(0, OFFSET_MOTOR_MAX);
    OM_FR_Slider->setValue(my_config.offsetMotorFR);
    payload.offsetMotorFR = 0;
    OM_FR_Label->setText("OM_FR Value : " + QString::number(OM_FR_Slider->value()));

    QLabel *OM_BR_Label = new QLabel("");
    QSlider *OM_BR_Slider = new QSlider(Qt::Horizontal);
    OM_BR_Slider->setRange(0, OFFSET_MOTOR_MAX);
    OM_BR_Slider->setValue(my_config.offsetMotorBR);
    payload.offsetMotorBR = 0;
    OM_BR_Label->setText("OM_BR Value : " + QString::number(OM_BR_Slider->value()));


    /***2.Communication***/
    QGroupBox *communicationGroupBox = new QGroupBox("Communication");
    QVBoxLayout *communicationVbox = new QVBoxLayout;
    QString UDPBoxName = QString("UDP (%1)").arg(ESP32_PORT);
    QCheckBox *UDPCheck = new QCheckBox(UDPBoxName);
    communicationVbox->addWidget(UDPCheck);
    communicationGroupBox->setLayout(communicationVbox);

    /***2.Security Box***/
    QGroupBox *securityGroupBox = new QGroupBox("Security");
    QCheckBox *FS_Check = new QCheckBox("FailSafe Switch (toggle on F)");
    QCheckBox *killCheck = new QCheckBox("Kill Switch (space bar)");
    FS_Check->setChecked(true);
    payload.failSafeSwitch = 1;
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
    payload.throttle = DSHOT_MIN;
    throttleLabel->setText("Throttle Value : " + QString::number(throttleSlider->value()));
    motorsVbox->addWidget(throttleLabel);
    motorsVbox->addWidget(throttleSlider);
    motorsGroupBox->setLayout(motorsVbox);


    /***2.Offset Motors***/
    QGroupBox *offsetsMotorsGroupBox = new QGroupBox("Offsets Motors");
    QGroupBox *offsetsMotorsGroupBoxLeft = new QGroupBox("");
    QGroupBox *offsetsMotorsGroupBoxRight = new QGroupBox("");
    QVBoxLayout *offsetsMotorsVboxLeft = new QVBoxLayout;
    QVBoxLayout *offsetsMotorsVboxRight = new QVBoxLayout;
    QHBoxLayout *offsetsMotorsHbox = new QHBoxLayout;

    offsetsMotorsVboxLeft->addWidget(OM_FL_Label);
    offsetsMotorsVboxLeft->addWidget(OM_FL_Slider);
    offsetsMotorsVboxLeft->addWidget(OM_BL_Label);
    offsetsMotorsVboxLeft->addWidget(OM_BL_Slider);
    offsetsMotorsGroupBoxLeft->setLayout(offsetsMotorsVboxLeft);

    offsetsMotorsVboxRight->addWidget(OM_FR_Label);
    offsetsMotorsVboxRight->addWidget(OM_FR_Slider);
    offsetsMotorsVboxRight->addWidget(OM_BR_Label);
    offsetsMotorsVboxRight->addWidget(OM_BR_Slider);
    offsetsMotorsGroupBoxRight->setLayout(offsetsMotorsVboxRight);

    offsetsMotorsHbox->addWidget(offsetsMotorsGroupBoxLeft);
    offsetsMotorsHbox->addWidget(offsetsMotorsGroupBoxRight);

    offsetsMotorsGroupBox->setLayout(offsetsMotorsHbox);

    /**1.Start**/
    QGroupBox *startGroupBox = new QGroupBox("Start");
    QVBoxLayout *startVbox = new QVBoxLayout;
    startVbox->addWidget(communicationGroupBox);
    startGroupBox->setLayout(startVbox);

    /**1.Controller**/
    QGroupBox *controllerGroupBox = new QGroupBox("Controller");
    controllerGroupBox->setEnabled(false); // Control non disponible tant que le serial n'est pas activé
    QVBoxLayout *controllerVbox = new QVBoxLayout;
    controllerVbox->addWidget(motorsGroupBox);
    controllerVbox->addWidget(offsetsMotorsGroupBox);
    controllerVbox->addWidget(securityGroupBox);
    controllerGroupBox->setLayout(controllerVbox);

    initConfigurationBox(&my_config);

    /* 0.Main Box */
    QGroupBox *mainGroupBox = new QGroupBox("");
    QHBoxLayout *mainHBox = new QHBoxLayout;
    mainHBox->addWidget(startGroupBox);
    mainHBox->addWidget(configurationGroupBox);
    mainHBox->addWidget(controllerGroupBox);
    mainGroupBox->setLayout(mainHBox);

    /* Display */
    layout.addWidget(mainGroupBox);
    window.setLayout(&layout);
    window.show();

    /* Connections */
    QShortcut *FS_Shortcut = new QShortcut(QKeySequence(Qt::Key_F), &window);
    QObject::connect(FS_Shortcut, &QShortcut::activated, [&]() { //Active/Désactive la limitation moteur (côté drone)
        FS_Check->setChecked(not FS_Check->isChecked());
        payload.failSafeSwitch = FS_Check->isChecked(); //Update value for UART
    });
    QObject::connect(FS_Check, &QCheckBox::toggled, [&](bool checked) {
        // Cette lambda est appelée à chaque clic sur la checkbox
        payload.failSafeSwitch = checked;
    });


    bool KS_enable = false;
    QShortcut *killShortcut = new QShortcut(QKeySequence(Qt::Key_Space), &window);
    QObject::connect(killShortcut, &QShortcut::activated, [&KS_enable, killCheck]() { //Désactive le drone
        if (KS_enable){
            killCheck->setChecked(true);
            payload.killSwitch = 1; //Update value for UART
        }
    });

    QObject::connect(killCheck, &QCheckBox::toggled, [&](bool checked){ // Désactive l'option KS une fois activée
        checked=checked; // no effect, avoid warning
        killCheck->setEnabled(false);
    });

    QObject::connect(throttleSlider, &QSlider::valueChanged, [throttleLabel](int value){ //Update la valeur de Throttle
        throttleLabel->setText("Throttle Value : " + QString::number(value));
        payload.throttle = value;  //Update value for UART
    });
//kpSpin, kiSpin, kdSpin, PID_GroupBox,
    QObject::connect(UDPCheck, &QCheckBox::toggled, [&, controllerGroupBox,UDPCheck, serial](bool checked){ // Active la manette et lance la communcation série
        checked = checked; // no effect, avoid warning
        controllerGroupBox->setEnabled(true);
        KS_enable = true;
        noFocusPID();
        UDPCheck->setEnabled(false);
        configure_UDP();
    });

    /*Update OFFSETS*/
    QObject::connect(OM_FL_Slider, &QSlider::valueChanged, [OM_FL_Label](int value){
        OM_FL_Label->setText("OM_FL Value : " + QString::number(value));
        my_config.offsetMotorFL = value;
        payload.offsetMotorFL = value;  //Update value for UART
    });

    QObject::connect(OM_BL_Slider, &QSlider::valueChanged, [OM_BL_Label](int value){
        OM_BL_Label->setText("OM_BL Value : " + QString::number(value));
        my_config.offsetMotorBL = value;
        payload.offsetMotorBL = value;  //Update value for UART
    });

    QObject::connect(OM_FR_Slider, &QSlider::valueChanged, [OM_FR_Label](int value){
        OM_FR_Label->setText("OM_FR Value : " + QString::number(value));
        my_config.offsetMotorFR = value;
        payload.offsetMotorFR = value;  //Update value for UART
    });

    QObject::connect(OM_BR_Slider, &QSlider::valueChanged, [OM_BR_Label](int value){
        OM_BR_Label->setText("OM_BR Value : " + QString::number(value));
        my_config.offsetMotorBR = value;
        payload.offsetMotorBR = value;  //Update value for UART
    });


    return app.exec();
}
