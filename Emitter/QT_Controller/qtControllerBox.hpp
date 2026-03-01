#ifndef __QT_CONTROLLER_BOX__H__
#define __QT_CONTROLLER_BOX__H__
#include <QGroupBox>
#include <iostream>
#include "controller.hpp"
extern QGroupBox* controllerGroupBox;
extern bool KS_enable;
void initControllerBox(QWidget* window, Controller* controller);

#endif
