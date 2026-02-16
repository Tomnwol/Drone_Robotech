#include <QGroupBox>
#include <QLabel>
#include <QProgressBar>
#include <QString>
#include <QVBoxLayout>
#include <QTimer>
#include "qtDroneBox.hpp"
#include "UDP.hpp"

QGroupBox *droneGroupBox = nullptr;
#define HEIGHT_BATTERY_CELL 5 //px

#define BATTERY_NO_DATA "#333333"
#define EMPTY_CELL_COLOR "#bbbbbb"

#define BATTERY_FULL_COLOR "#54D651"
#define BATTERY_OK_COLOR "#82D651"
#define BATTERY_HALF_COLOR "#D2D651"
#define BATTERY_LOW_COLOR "#EE8311"
#define BATTERY_STOP_COLOR "#C72610"

#define BATTERY_CELL_COUNT 5

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

void updateDroneBox(){
    updateColorCells( droneBattery );
    updateValueLabel( droneBatteryLabel, droneBattery );
}

void initDroneBox(){

    droneGroupBox = new QGroupBox("Drone");
    droneBatteryLabel = new QLabel("");

    //droneBatteryLabel->setText("Battery % : " + QString::number(batteryValue));

    QVBoxLayout *droneVbox = new QVBoxLayout;
    droneVbox->addWidget(droneBatteryLabel);

    // Création des cellules
    for(int i = 0; i < BATTERY_CELL_COUNT; i++) {

        droneBatteryCells[i] = new QWidget();
        droneBatteryCells[i]->setMinimumHeight(HEIGHT_BATTERY_CELL);
        //droneBatteryCells[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

        droneVbox->addWidget(droneBatteryCells[i]);
    }

    //updateDroneBox();
    updateTimer = new QTimer();
    updateTimer->setInterval(50); // 100 ms → 20 Hz
    QObject::connect(updateTimer, &QTimer::timeout, []() {
        updateDroneBox();
    });
    updateTimer->start();
    droneGroupBox->setLayout(droneVbox);
}

