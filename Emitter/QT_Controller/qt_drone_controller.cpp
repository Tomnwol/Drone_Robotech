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
    QString initialValuesName = "initialValues.ini";
    Config my_config = loadConfig(initialValuesName);

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


    /***2.Configuration->PID SpinBox***/
    QGroupBox *PID_GroupBox = new QGroupBox("PID Values");
    QVBoxLayout *vbox_PID = new QVBoxLayout;
    QLabel *kpLabel = new QLabel("Kp:");
    QDoubleSpinBox *kpSpin = new QDoubleSpinBox();
    kpSpin->setRange(1.0, 100.0);    // plage de valeurs
    kpSpin->setSingleStep(0.5);      // incrément à chaque flèche
    kpSpin->setValue(my_config.Kp);           // valeur par défaut
    kpSpin->setDecimals(2);          // nombre de décimales affichées
    payload.KP = my_config.Kp * PID_MULTIPLICATOR;

    QLabel *kiLabel = new QLabel("Ki:");
    QDoubleSpinBox *kiSpin = new QDoubleSpinBox();
    kiSpin->setRange(0.0, 100.0);    // plage de valeurs
    kiSpin->setSingleStep(0.5);      // incrément à chaque flèche
    kiSpin->setValue(my_config.Ki);           // valeur par défaut
    kiSpin->setDecimals(2);          // nombre de décimales affichées
    payload.KI = my_config.Ki * PID_MULTIPLICATOR;

    QLabel *kdLabel = new QLabel("Kd:");
    QDoubleSpinBox *kdSpin = new QDoubleSpinBox();
    kdSpin->setRange(0.0, 150.0);    // plage de valeurs
    kdSpin->setSingleStep(0.5);      // incrément à chaque flèche
    kdSpin->setValue(my_config.Kd);           // valeur par défaut
    kdSpin->setDecimals(2);          // nombre de décimales affichées
    payload.KD = my_config.Kd * PID_MULTIPLICATOR;

    vbox_PID->addWidget(kpLabel);
    vbox_PID->addWidget(kpSpin);
    vbox_PID->addWidget(kiLabel);
    vbox_PID->addWidget(kiSpin);
    vbox_PID->addWidget(kdLabel);
    vbox_PID->addWidget(kdSpin);
    PID_GroupBox->setLayout(vbox_PID);

    /***2.Configuration->SaveParameters***/
    QGroupBox *saveGroupBox = new QGroupBox("Save Config");
    QVBoxLayout *saveVBox = new QVBoxLayout;
    QPushButton *save_button = new QPushButton();
    save_button->setText("SAVE");
    saveVBox->addWidget(save_button);
    saveGroupBox->setLayout(saveVBox);
    /**1.Start**/
    QGroupBox *startGroupBox = new QGroupBox("Start");
    QVBoxLayout *startVbox = new QVBoxLayout;
    startVbox->addWidget(communicationGroupBox);
    startGroupBox->setLayout(startVbox);

    /**1.Configuration**/
    QGroupBox *configurationGroupBox = new QGroupBox("Configuration");
    QVBoxLayout *configurationVbox = new QVBoxLayout;
    configurationVbox->addWidget(PID_GroupBox);
    configurationVbox->addWidget(saveGroupBox);
    configurationGroupBox->setLayout(configurationVbox);

    /**1.Controller**/
    QGroupBox *controllerGroupBox = new QGroupBox("Controller");
    controllerGroupBox->setEnabled(false); // Control non disponible tant que le serial n'est pas activé
    QVBoxLayout *controllerVbox = new QVBoxLayout;
    controllerVbox->addWidget(motorsGroupBox);
    controllerVbox->addWidget(offsetsMotorsGroupBox);
    controllerVbox->addWidget(securityGroupBox);
    controllerGroupBox->setLayout(controllerVbox);

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

    QObject::connect(UDPCheck, &QCheckBox::toggled, [&, kpSpin, kiSpin, kdSpin, PID_GroupBox, controllerGroupBox,UDPCheck, serial](bool checked){ // Active la manette et lance la communcation série
        checked = checked; // no effect, avoid warning
        controllerGroupBox->setEnabled(true);
        KS_enable = true;
        kpSpin->setFocusPolicy(Qt::NoFocus);
        kiSpin->setFocusPolicy(Qt::NoFocus);
        kdSpin->setFocusPolicy(Qt::NoFocus);
        UDPCheck->setEnabled(false);
        configure_UDP();
    });


    QObject::connect(kpSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&my_config](double value){
                         my_config.Kp = value;
                         payload.KP = (uint16_t)(value * PID_MULTIPLICATOR);
                     }
    );

    QObject::connect(kiSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&my_config](double value){
                         my_config.Ki = value;
                         payload.KI = (uint16_t)(value * PID_MULTIPLICATOR);
                     }
    );

    QObject::connect(kdSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&my_config](double value){
                         my_config.Kd = value;
                         payload.KD = (uint16_t)(value * PID_MULTIPLICATOR);
                     }
    );

    /*Update OFFSETS*/
    QObject::connect(OM_FL_Slider, &QSlider::valueChanged, [OM_FL_Label, &my_config](int value){
        OM_FL_Label->setText("OM_FL Value : " + QString::number(value));
        my_config.offsetMotorFL = value;
        payload.offsetMotorFL = value;  //Update value for UART
    });

    QObject::connect(OM_BL_Slider, &QSlider::valueChanged, [OM_BL_Label, &my_config](int value){
        OM_BL_Label->setText("OM_BL Value : " + QString::number(value));
        my_config.offsetMotorBL = value;
        payload.offsetMotorBL = value;  //Update value for UART
    });

    QObject::connect(OM_FR_Slider, &QSlider::valueChanged, [OM_FR_Label, &my_config](int value){
        OM_FR_Label->setText("OM_FR Value : " + QString::number(value));
        my_config.offsetMotorFR = value;
        payload.offsetMotorFR = value;  //Update value for UART
    });

    QObject::connect(OM_BR_Slider, &QSlider::valueChanged, [OM_BR_Label, &my_config](int value){
        OM_BR_Label->setText("OM_BR Value : " + QString::number(value));
        my_config.offsetMotorBR = value;
        payload.offsetMotorBR = value;  //Update value for UART
    });

    QObject::connect(save_button, &QPushButton::clicked, [&initialValuesName, &my_config](){
        saveConfig(initialValuesName, my_config);
        std::cout << "Configuration saved !\n";
    });

    return app.exec();
}
