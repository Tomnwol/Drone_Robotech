#include <QWidget>
#include <QGroupBox>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QString>
#include <QHBoxLayout>
#include <QShortcut>
#include <QKeyEvent>
#include "qtControllerBox.hpp"
#include "configuration.hpp"
#include "UDP.hpp"
#define OFFSET_MOTOR_MAX 100

#define DSHOT_MIN 48
#define DSHOT_MAX 2047

#define ANGLE_MAX 1000  // (yaw/pitch/roll)


QGroupBox *controllerGroupBox = nullptr;
bool KS_enable = false;
void initControllerBox(QWidget* window){

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

    /***2.Security Box***/
    QGroupBox *securityGroupBox = new QGroupBox("Security");
    QCheckBox *FS_Check = new QCheckBox("FailSafe Switch (toggle on F)");
    QCheckBox *killCheck = new QCheckBox("Kill Switch (space bar)", securityGroupBox);
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
    QLabel *yawLabel = new QLabel("");
    QLabel *rollLabel = new QLabel("");
    QLabel *pitchLabel = new QLabel("");
    QSlider *throttleSlider = new QSlider(Qt::Horizontal);
    QSlider *yawSlider = new QSlider(Qt::Horizontal);
    QSlider *rollSlider = new QSlider(Qt::Horizontal);
    QSlider *pitchSlider = new QSlider(Qt::Horizontal);

    throttleSlider->setRange(DSHOT_MIN, DSHOT_MAX);
    throttleSlider->setValue(DSHOT_MIN);
    yawSlider->setRange(-ANGLE_MAX, +ANGLE_MAX);
    yawSlider->setValue(0);
    rollSlider->setRange(-ANGLE_MAX, +ANGLE_MAX);
    rollSlider->setValue(0);
    pitchSlider->setRange(-ANGLE_MAX, +ANGLE_MAX);
    pitchSlider->setValue(0);

    payload.yaw = 0;
    payload.roll = 0;
    payload.pitch = 0;
    payload.throttle = DSHOT_MIN;
    throttleLabel->setText("Throttle Value : " + QString::number(throttleSlider->value()));
    yawLabel->setText("Yaw Value : " + QString::number(yawSlider->value()));
    rollLabel->setText("Roll Value : " + QString::number(rollSlider->value()));
    pitchLabel->setText("Pitch Value : " + QString::number(pitchSlider->value()));
    motorsVbox->addWidget(throttleLabel);
    motorsVbox->addWidget(throttleSlider);
    motorsVbox->addWidget(yawLabel);
    motorsVbox->addWidget(yawSlider);
    motorsVbox->addWidget(rollLabel);
    motorsVbox->addWidget(rollSlider);
    motorsVbox->addWidget(pitchLabel);
    motorsVbox->addWidget(pitchSlider);
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

    /**1.Controller**/
    controllerGroupBox = new QGroupBox("Controller");
    controllerGroupBox->setEnabled(false); // Control non disponible tant que le serial n'est pas activé
    QVBoxLayout *controllerVbox = new QVBoxLayout;
    controllerVbox->addWidget(motorsGroupBox);
    controllerVbox->addWidget(offsetsMotorsGroupBox);
    controllerVbox->addWidget(securityGroupBox);
    controllerGroupBox->setLayout(controllerVbox);

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

    QShortcut *FS_Shortcut = new QShortcut(QKeySequence(Qt::Key_F), window);
    QObject::connect(FS_Shortcut, &QShortcut::activated, FS_Check, [FS_Check]() { //Active/Désactive la limitation moteur (côté drone)
        QSignalBlocker blocker(FS_Check);
        FS_Check->setChecked(not FS_Check->isChecked());
        payload.failSafeSwitch = FS_Check->isChecked(); //Update value for UART

    });
    QObject::connect(FS_Check, &QCheckBox::toggled, FS_Check, [FS_Check](bool checked) {
        // Cette lambda est appelée à chaque clic sur la checkbox
        QSignalBlocker blocker(FS_Check);
        payload.failSafeSwitch = checked;

    });


    QShortcut *killShortcut = new QShortcut(QKeySequence(Qt::Key_Space), window);
    QObject::connect(killShortcut,
                     &QShortcut::activated,
                     killCheck,
                     [killCheck]() {
                         if (KS_enable){
                             QSignalBlocker blocker(killCheck);
                             killCheck->setChecked(true);
                             killCheck->setEnabled(false);
                             payload.killSwitch = 1;
                         }
                     });

    QObject::connect(killCheck, &QCheckBox::toggled, killCheck, [killCheck](bool checked){ // Désactive l'option KS une fois activée
        checked=checked; // no effect, avoid warning
        QSignalBlocker blocker(killCheck);
        killCheck->setEnabled(false);
        payload.killSwitch = 1;
    });

    QObject::connect(throttleSlider, &QSlider::valueChanged, [throttleLabel](int value){ //Update la valeur de Throttle
        throttleLabel->setText("Throttle Value : " + QString::number(value));
        payload.throttle = value;  //Update value for UART
    });

    QObject::connect(yawSlider, &QSlider::valueChanged, [yawLabel](int value){ //Update la valeur de Throttle
        yawLabel->setText("Yaw Value : " + QString::number(value));
        payload.yaw = value;  //Update value for UART
    });

    QObject::connect(rollSlider, &QSlider::valueChanged, [rollLabel](int value){ //Update la valeur de Throttle
        rollLabel->setText("Roll Value : " + QString::number(value));
        payload.roll = value;  //Update value for UART
    });

    QObject::connect(pitchSlider, &QSlider::valueChanged, [pitchLabel](int value){ //Update la valeur de Throttle
        pitchLabel->setText("Pitch Value : " + QString::number(value));
        payload.pitch = value;  //Update value for UART
    });
}
