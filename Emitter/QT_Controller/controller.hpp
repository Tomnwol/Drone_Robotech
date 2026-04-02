#ifndef __CONTROLLER__H__
#define __CONTROLLER__H__
#include <QObject>
#include <QSocketNotifier>
#include <libevdev/libevdev.h>
extern int ButtonA;
extern int ButtonB;
extern int ButtonY;
extern int ButtonX;
extern int ButtonXBOX;
extern int ButtonMENU;
extern int ButtonBackLeft;
extern int ButtonBackRight;
extern double throttleAxis;
extern double throttleValue;
extern int yawAxis;
//extern int LeftTrigger;
extern int RightTrigger;
extern int pitchAxis;
extern int rollAxis;

extern int valueMenu;

extern bool isControllerFound;
int clampInt(int value, int min, int max);
double clampDouble(double value, double min, double max) ;
class Controller : public QObject
{
    Q_OBJECT

public:
    explicit Controller(QObject* parent = nullptr);
    void initController(const std::string& expectedName);

private slots:
    void readController();

private:
    int fd = -1;
    libevdev* dev = nullptr;
    QSocketNotifier* notifier = nullptr;
};
#endif
