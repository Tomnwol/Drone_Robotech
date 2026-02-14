#ifndef __QT_BOX_CONFIGURATION__H__
#define __QT_BOX_CONFIGURATION__H__
#include <QGroupBox>
#include <iostream>
extern QGroupBox* configurationGroupBox;
void initConfigurationBox(Config* config);
void noFocusPID();

#endif
