#include <QSurfaceFormat>
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
#include "qtStartBox.hpp"
#include "qtBoxConfiguration.hpp"
#include "qtControllerBox.hpp"
#include "qtDroneBox.hpp"
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
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(4, 3); // OpenGL 4.3 safe
    format.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(format);
    QApplication app(argc, argv);
    QWidget window;
    QSerialPort* serial = new QSerialPort(&window);
    window.setWindowTitle("Drone Controller Interface");
    QVBoxLayout layout;
    my_config = loadConfig(INITIAL_VALUES_PATH);

    initStartBox(serial);
    initConfigurationBox(&my_config);
    initControllerBox(&window);
    initDroneBox();
    /* 0.Main Box */
    QGroupBox *mainGroupBox = new QGroupBox("");
    QHBoxLayout *mainHBox = new QHBoxLayout;
    mainHBox->addWidget(startGroupBox);
    mainHBox->addWidget(configurationGroupBox);
    mainHBox->addWidget(controllerGroupBox);
    mainHBox->addWidget(droneGroupBox);
    mainGroupBox->setLayout(mainHBox);

    /* Display */
    layout.addWidget(mainGroupBox);
    window.setLayout(&layout);
    window.show();

    /* Connections */

    return app.exec();
}
