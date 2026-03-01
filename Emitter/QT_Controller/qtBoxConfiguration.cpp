#include <QVBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QPushButton>
#include "UDP.hpp"
#include "configuration.hpp"
#include "qtBoxConfiguration.hpp"

#define OFFSET_MOTOR_MAX 100

QDoubleSpinBox *kpSpin;
QDoubleSpinBox *kiSpin;
QDoubleSpinBox *kdSpin;

QGroupBox *configurationGroupBox = nullptr; // WARNING : ne pas intialiser en global sinon crash

void initConfigurationBox(Config* config){
    configurationGroupBox = new QGroupBox("Configuration");

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
    configurationVbox->addWidget(offsetsMotorsGroupBox);
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


}

void noFocusPID(){
    kpSpin->setFocusPolicy(Qt::NoFocus);
    kiSpin->setFocusPolicy(Qt::NoFocus);
    kdSpin->setFocusPolicy(Qt::NoFocus);
}
