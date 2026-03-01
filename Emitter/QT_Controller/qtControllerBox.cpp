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
#include <QTimer>
#include "qtControllerBox.hpp"
#include "configuration.hpp"

#include "UDP.hpp"


#define DSHOT_MIN 48
#define DSHOT_MAX 2047

#define ANGLE_MAX 1000  // (yaw/pitch/roll)

Controller* local_controller;
QGroupBox *controllerGroupBox = nullptr;
QTimer *timerControllerUpdate = nullptr;
bool KS_enable = false;

bool lastButtonA = false;
bool lastButtonB = false;
bool lastButtonY = false;
bool lastButtonX = false;

void initControllerBox(QWidget* window, Controller* controller){
    local_controller = controller;
    /***2.Security Box***/
    QGroupBox *securityGroupBox = new QGroupBox("Security");
    QCheckBox *FS_Check = new QCheckBox("FailSafe Switch (KeyF, X or Y)");
    QCheckBox *killCheck = new QCheckBox("Kill Switch (KeySpaceBar, A or B)", securityGroupBox);
    killCheck->setStyleSheet(
        "QCheckBox::indicator:checked {"
        "background-color: #aa0000;"
        "border: 1px solid black;"
        "}"
    );
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




    /**1.Controller**/
    controllerGroupBox = new QGroupBox("Controller");
    controllerGroupBox->setStyleSheet("QGroupBox { font-family: 'DejaVu Sans Mono'; font-size: 16px;  font-weight: bold; }");
    controllerGroupBox->setEnabled(false); // Control non disponible tant que le serial n'est pas activé
    QVBoxLayout *controllerVbox = new QVBoxLayout;

    motorsGroupBox->setStyleSheet("QGroupBox { font-weight: normal; }");
    securityGroupBox->setStyleSheet("QGroupBox { font-weight: normal; }");
    controllerVbox->addWidget(motorsGroupBox);
    controllerVbox->addWidget(securityGroupBox);
    controllerGroupBox->setLayout(controllerVbox);


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
        std::cout << "KILLSWITCHED" << std::endl;
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

    timerControllerUpdate = new QTimer();
    timerControllerUpdate->setInterval(20); // 20 ms → 50 Hz

    QObject::connect(timerControllerUpdate, &QTimer::timeout, [throttleSlider, yawSlider, rollSlider, pitchSlider, killCheck, FS_Check]() {
        if (!throttleSlider->isSliderDown()){
            throttleSlider->setValue(throttleAxis);
        }else{
            throttleAxis = throttleSlider->value();
        }

        if (!yawSlider->isSliderDown()){
            yawSlider->setValue(CWRotationTrigger - CCWRotationTrigger);
        }

        if (!rollSlider->isSliderDown()){
            rollSlider->setValue(rollAxis);
        }else{
            rollAxis = rollSlider->value();
        }

        if (!pitchSlider->isSliderDown()){
            pitchSlider->setValue(pitchAxis);
        }else{
            pitchAxis = pitchSlider->value();
        }

        if ( (ButtonA && !lastButtonA) | (ButtonB && !lastButtonB) ){
            if (KS_enable){
                killCheck->setChecked(true);
            }
        }else if( (ButtonY && !lastButtonY) | (ButtonX && !lastButtonX) ){
            FS_Check->setChecked(not FS_Check->isChecked());
        }

        lastButtonA = ButtonA;
        lastButtonB = ButtonB;
        lastButtonY = ButtonY;
        lastButtonX = ButtonX;

    });
    timerControllerUpdate->start();

}
