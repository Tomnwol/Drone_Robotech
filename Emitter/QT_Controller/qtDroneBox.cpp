#include <QGroupBox>
#include <QLabel>
#include <QProgressBar>
#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QSlider>
#include <QQuickWidget>
#include <QQuickItem>
#include "qtDroneBox.hpp"
#include "UDP.hpp"

QGroupBox *droneGroupBox = nullptr;
QQuickWidget *quickWidget = nullptr;
QObject *root = nullptr;
#define HEIGHT_BATTERY_CELL 5 //px

#define BATTERY_NO_DATA "#333333"
#define EMPTY_CELL_COLOR "#bbbbbb"

#define BATTERY_FULL_COLOR "#54D651"
#define BATTERY_OK_COLOR "#82D651"
#define BATTERY_HALF_COLOR "#D2D651"
#define BATTERY_LOW_COLOR "#EE8311"
#define BATTERY_STOP_COLOR "#C72610"

#define BATTERY_CELL_COUNT 5

#define DSHOT_MIN 48
#define DSHOT_MAX 2047
#define PI 3.1415

QWidget* droneBatteryCells[BATTERY_CELL_COUNT];
QProgressBar *droneBatteryProgressBar;
QLabel *droneBatteryLabel;
QTimer* updateTimer = nullptr;

QString getColorByValue(float value){
    QString color;

    if(value <= 100 && value > 80){
    	color = BATTERY_FULL_COLOR;
    }else if(value > 60){
    	color = BATTERY_OK_COLOR;
    }else if(value > 40){
    	color = BATTERY_HALF_COLOR;
    }else if(value > 20){
    	color = BATTERY_LOW_COLOR;
    }else if(value >= 0){
    	color = BATTERY_STOP_COLOR;
    }else{
    	color = BATTERY_NO_DATA;
    }
    return color;
}

void updateColorCells(uint8_t value) {
    QString current_color = getColorByValue(value);
    int activeCells = (value * BATTERY_CELL_COUNT) / 100 + 1;

    for(int i = 0; i < BATTERY_CELL_COUNT; i++) {
        if (value > 100){
            droneBatteryCells[i]->setStyleSheet("background-color: " + QString(BATTERY_NO_DATA) + ";");
            continue;
        }
        // Remplissage depuis le bas
        if(i >= BATTERY_CELL_COUNT - activeCells) {
            droneBatteryCells[i]->setStyleSheet("background-color: " + current_color + ";");
        } else {
            droneBatteryCells[i]->setStyleSheet("background-color: " + QString(EMPTY_CELL_COLOR) + ";");
        }
    }
}

void updateValueLabel(QLabel *batteryLabel, uint8_t value) {

    if (value <= 100){
        batteryLabel->setText("Battery % : " + QString::number(value));
    }else{
        batteryLabel->setText("--NO DATA--");
    }

}
//float test = 4;
QSlider *yawSlider = nullptr;
QSlider *rollSlider = nullptr;
QSlider *pitchSlider = nullptr;
QSlider *FL_Slider = nullptr;
QSlider *BL_Slider = nullptr;
QSlider *FR_Slider = nullptr;
QSlider *BR_Slider = nullptr;

void updateSlider() {
    yawSlider->setValue(droneYaw);
    rollSlider->setValue(droneRoll);
    pitchSlider->setValue(dronePitch);
    FL_Slider->setValue(droneMotorFL);
    BL_Slider->setValue(droneMotorBL);
    FR_Slider->setValue(droneMotorFR);
    BR_Slider->setValue(droneMotorBR);
}

void updateDroneBox(){
    updateColorCells( droneBattery );
    updateValueLabel( droneBatteryLabel, droneBattery );
    updateSlider();
    //test++;
    QMetaObject::invokeMethod(root, "setOrientation",
                              Q_ARG(QVariant, 0),
                              Q_ARG(QVariant, rollSlider->value()),
                              Q_ARG(QVariant, pitchSlider->value()));

}



void initDroneBox(){
    /* Drone Motors */
    /***3. Attitude ***/

    QLabel *yawLabel = new QLabel("");
    yawSlider = new QSlider(Qt::Horizontal);
    yawSlider->setRange(-180, 180);
    yawSlider->setValue(0);
    yawLabel->setText("Yaw : " + QString::number(yawSlider->value()));

    QLabel *rollLabel = new QLabel("");
    rollSlider = new QSlider(Qt::Horizontal);
    rollSlider->setRange(-180, 180);
    rollSlider->setValue(0);
    rollLabel->setText("Roll : " + QString::number(rollSlider->value()));

    QLabel *pitchLabel = new QLabel("");
    pitchSlider = new QSlider(Qt::Horizontal);
    pitchSlider->setRange(-180, 180);
    pitchSlider->setValue(0);
    pitchLabel->setText("Pitch : " + QString::number(pitchSlider->value()));

    QGroupBox *attitudesGroupBox = new QGroupBox("");
    QGroupBox *attitudePBGroupBox = new QGroupBox("Attitude measured");
    QVBoxLayout *attitudePBVbox = new QVBoxLayout;
    QHBoxLayout *attitudesHbox = new QHBoxLayout;
    attitudePBVbox->addWidget(yawLabel);
    attitudePBVbox->addWidget(yawSlider);
    attitudePBVbox->addWidget(rollLabel);
    attitudePBVbox->addWidget(rollSlider);
    attitudePBVbox->addWidget(pitchLabel);
    attitudePBVbox->addWidget(pitchSlider);
    attitudePBGroupBox->setLayout(attitudePBVbox);

    /****3.Left motors****/
    QLabel *FL_Label = new QLabel("");
    FL_Slider = new QSlider(Qt::Horizontal);
    FL_Slider->setRange(0, DSHOT_MAX);
    FL_Slider->setValue(DSHOT_MIN);
    FL_Label->setText("FL Value : " + QString::number(FL_Slider->value()));

    QLabel *BL_Label = new QLabel("");
    BL_Slider = new QSlider(Qt::Horizontal);
    BL_Slider->setRange(0, DSHOT_MAX);
    BL_Slider->setValue(DSHOT_MIN);
    BL_Label->setText("BL Value : " + QString::number(BL_Slider->value()));

    /****3.Right motors****/
    QLabel *FR_Label = new QLabel("");
    FR_Slider = new QSlider(Qt::Horizontal);
    FR_Slider->setRange(0, DSHOT_MAX);
    FR_Slider->setValue(DSHOT_MIN);
    FR_Label->setText("FR Value : " + QString::number(FR_Slider->value()));

    QLabel *BR_Label = new QLabel("");
    BR_Slider = new QSlider(Qt::Horizontal);
    BR_Slider->setRange(0, DSHOT_MAX);
    BR_Slider->setValue(DSHOT_MIN);
    BR_Label->setText("BR Value : " + QString::number(BR_Slider->value()));

    droneGroupBox = new QGroupBox("Drone");
    droneGroupBox->setStyleSheet("QGroupBox { font-family: 'DejaVu Sans Mono'; font-size: 16px;  font-weight: bold; }");
    droneBatteryLabel = new QLabel("");

    QVBoxLayout *droneVbox = new QVBoxLayout;
    droneVbox->addWidget(droneBatteryLabel);

    // Création des cellules
    for(int i = 0; i < BATTERY_CELL_COUNT; i++) {

        droneBatteryCells[i] = new QWidget();
        droneBatteryCells[i]->setMinimumHeight(HEIGHT_BATTERY_CELL);
        droneBatteryCells[i]->setFixedWidth(50);
        droneBatteryCells[i]->setFixedHeight(20);
        droneBatteryCells[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

        droneVbox->addWidget(droneBatteryCells[i]);
    }

    //updateDroneBox();
    updateTimer = new QTimer();
    updateTimer->setInterval(50); // 100 ms → 20 Hz
    QObject::connect(updateTimer, &QTimer::timeout, []() {
        updateDroneBox();

    });
    updateTimer->start();

    /***2. Motors***/
    QGroupBox *MotorsGroupBox = new QGroupBox("Motors");
    QGroupBox *MotorsGroupBoxLeft = new QGroupBox("");
    QGroupBox *MotorsGroupBoxRight = new QGroupBox("");
    QVBoxLayout *MotorsVboxLeft = new QVBoxLayout;
    QVBoxLayout *MotorsVboxRight = new QVBoxLayout;
    QHBoxLayout *MotorsHbox = new QHBoxLayout;

    MotorsVboxLeft->addWidget(FL_Label);
    MotorsVboxLeft->addWidget(FL_Slider);
    MotorsVboxLeft->addWidget(BL_Label);
    MotorsVboxLeft->addWidget(BL_Slider);
    MotorsGroupBoxLeft->setLayout(MotorsVboxLeft);

    MotorsVboxRight->addWidget(FR_Label);
    MotorsVboxRight->addWidget(FR_Slider);
    MotorsVboxRight->addWidget(BR_Label);
    MotorsVboxRight->addWidget(BR_Slider);
    MotorsGroupBoxRight->setLayout(MotorsVboxRight);

    MotorsHbox->addWidget(MotorsGroupBoxLeft);
    MotorsHbox->addWidget(MotorsGroupBoxRight);

    MotorsGroupBox->setLayout(MotorsHbox);
    droneVbox->addWidget(MotorsGroupBox);
    //droneVbox->addWidget(attitudePBGroupBox);

    /*  3D */
    QGroupBox *attitude3DGroupBox = new QGroupBox("");
    quickWidget = new QQuickWidget(attitude3DGroupBox);
    quickWidget->setResizeMode(QQuickWidget::SizeRootObjectToView);
    quickWidget->setSource(QUrl("cube.qml"));

    attitudesHbox->addWidget(attitudePBGroupBox);
    attitudesHbox->addWidget(quickWidget);
    attitudesGroupBox->setLayout(attitudesHbox);

    root = quickWidget->rootObject();

    MotorsGroupBox->setStyleSheet("QGroupBox { font-weight: normal; }");
    attitudesGroupBox->setStyleSheet("QGroupBox { font-weight: normal; }");

    MotorsGroupBox->setEnabled(false);
    attitudesGroupBox->setEnabled(false);

    droneVbox->addWidget(attitudesGroupBox);
    //droneGroupBox->setLayout(MotorsHbox);
    droneGroupBox->setLayout(droneVbox);

    QObject::connect(yawSlider, &QSlider::valueChanged, [yawLabel](int value){
        yawLabel->setText("Yaw : " + QString::number(value));
    });

    QObject::connect(rollSlider, &QSlider::valueChanged, [rollLabel](int value){
        rollLabel->setText("Roll : " + QString::number(value));
    });

    QObject::connect(pitchSlider, &QSlider::valueChanged, [pitchLabel](int value){
        pitchLabel->setText("Pitch : " + QString::number(value));
    });

}

