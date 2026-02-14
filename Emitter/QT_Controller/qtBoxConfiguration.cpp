#include <QVBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>
#include "UDP.hpp"
#include "configuration.hpp"
#include "qtBoxConfiguration.hpp"
QDoubleSpinBox *kpSpin;
QDoubleSpinBox *kiSpin;
QDoubleSpinBox *kdSpin;

QGroupBox *configurationGroupBox = nullptr; // WARNING : ne pas intialiser en global sinon crash

void initConfigurationBox(Config* config){
    configurationGroupBox = new QGroupBox("Configuration");
    /***2.Configuration->PID SpinBox***/
    QGroupBox *PID_GroupBox = new QGroupBox("PID Values");
    QVBoxLayout *vbox_PID = new QVBoxLayout;
    QLabel *kpLabel = new QLabel("Kp:");
    kpSpin = new QDoubleSpinBox();
    kpSpin->setRange(1.0, 100.0);    // plage de valeurs
    kpSpin->setSingleStep(0.5);      // incrément à chaque flèche
    kpSpin->setValue(config->Kp);           // valeur par défaut
    kpSpin->setDecimals(2);          // nombre de décimales affichées
    payload.KP = config->Kp * PID_MULTIPLICATOR;

    QLabel *kiLabel = new QLabel("Ki:");
    kiSpin = new QDoubleSpinBox();
    kiSpin->setRange(0.0, 100.0);    // plage de valeurs
    kiSpin->setSingleStep(0.5);      // incrément à chaque flèche
    kiSpin->setValue(config->Ki);           // valeur par défaut
    kiSpin->setDecimals(2);          // nombre de décimales affichées
    payload.KI = config->Ki * PID_MULTIPLICATOR;

    QLabel *kdLabel = new QLabel("Kd:");
    kdSpin = new QDoubleSpinBox();
    kdSpin->setRange(0.0, 150.0);    // plage de valeurs
    kdSpin->setSingleStep(0.5);      // incrément à chaque flèche
    kdSpin->setValue(config->Kd);           // valeur par défaut
    kdSpin->setDecimals(2);          // nombre de décimales affichées
    payload.KD = config->Kd * PID_MULTIPLICATOR;

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

    /**1.Configuration**/
    //QGroupBox *configurationGroupBox = new QGroupBox("Configuration");
    QVBoxLayout *configurationVbox = new QVBoxLayout;
    configurationVbox->addWidget(PID_GroupBox);
    configurationVbox->addWidget(saveGroupBox);
    configurationGroupBox->setLayout(configurationVbox);

    QObject::connect(kpSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                    [config](double value){
                        config->Kp = value;
                        payload.KP = (uint16_t)(value * PID_MULTIPLICATOR);
                    }
    );

    QObject::connect(kiSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                    [config](double value){
                        config->Ki = value;
                        payload.KI = (uint16_t)(value * PID_MULTIPLICATOR);
                    }
    );

    QObject::connect(kdSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                    [config](double value){
                        config->Kd = value;
                        payload.KD = (uint16_t)(value * PID_MULTIPLICATOR);
                    }
    );

    QObject::connect(save_button, &QPushButton::clicked, [](){
        saveConfig(INITIAL_VALUES_PATH);
        std::cout << "Configuration saved !\n";
    });
}

void noFocusPID(){
    kpSpin->setFocusPolicy(Qt::NoFocus);
    kiSpin->setFocusPolicy(Qt::NoFocus);
    kdSpin->setFocusPolicy(Qt::NoFocus);
}
