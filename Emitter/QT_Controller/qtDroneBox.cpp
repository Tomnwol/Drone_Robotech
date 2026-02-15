#include <QGroupBox>
#include <QLabel>
#include <QSlider>
#include <QString>
#include <QVBoxLayout>
#include "qtDroneBox.hpp"

QGroupBox *droneGroupBox = nullptr;

void initDroneBox(){
    droneGroupBox = new QGroupBox("Drone");
    QLabel *droneBatteryLabel = new QLabel("Battery");
    QSlider *droneBatterySlider = new QSlider(Qt::Vertical);
    droneBatterySlider->setRange(0, 100);
    droneBatterySlider->setValue(70);
    droneBatterySlider->setEnabled(false);
    droneBatterySlider->setStyleSheet(
        "QSlider::groove:vertical { width: 50px; background: rgb(70,70,70); border-radius: 5px; }"
        "QSlider::add-page:vertical { background: rgb(0,200,0); border-radius: 5px; }"
        "QSlider::sub-page:vertical { background: rgb(70,70,70); border-radius: 5px; }"   // <-- vert
        "QSlider::handle:vertical { width: 50px; height: 20px; background: solid black; border: 3px solid black; margin: 0 -5px; border-radius: 5px; }"
    );
    droneBatteryLabel->setText("Drone Battery % : " + QString::number(droneBatterySlider->value()));
    QVBoxLayout *droneVbox = new QVBoxLayout;
    droneVbox->addWidget(droneBatteryLabel);
    droneVbox->addWidget(droneBatterySlider);
    droneGroupBox->setLayout(droneVbox);
}
