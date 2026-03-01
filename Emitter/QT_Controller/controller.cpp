#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <libevdev/libevdev.h>
#include <QTimer>
#include <stdlib.h>
#include "controller.hpp"

#define CODE_BUTTON_A 304
#define CODE_BUTTON_B 305
#define CODE_BUTTON_Y 308
#define CODE_BUTTON_X 307
#define CODE_BUTTON_XBOX 316
#define CODE_THROTTLE_AXIS 1
#define CODE_CW_ROTATION_TRIGGER 5
#define CODE_CCW_ROTATION_TRIGGER 2
#define CODE_PITCH_AXIS 4
#define CODE_ROLL_AXIS 3

#define JOYSTICK_RESOLUTION 1000

int ButtonA = 0;
int ButtonB = 0;
int ButtonY = 0;
int ButtonX = 0;
int ButtonXBOX = 0;
int throttleAxis = 0;
int throttleValue = 0;
int CWRotationTrigger = 0;
int CCWRotationTrigger = 0;
int pitchAxis = 0;
int rollAxis = 0;

bool isControllerFound = false;

int remapInt(int value, int inMin, int inMax, int outMin, int outMax) {
    if (inMax == inMin) return outMin; // éviter division par zéro
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}


int clampInt(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

std::string findControllerDevice(const std::string& expectedName) {
    for (int i = 0; i < 32; ++i) { // test /dev/input/event0 à event31
        std::string path = "/dev/input/event" + std::to_string(i);
        int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;

        libevdev* dev = nullptr;
        if (libevdev_new_from_fd(fd, &dev) >= 0) {
            std::string name = libevdev_get_name(dev);
            if (name == expectedName) {
                libevdev_free(dev);
                close(fd);
                return path; // on retourne le chemin du device trouvé
            }
            libevdev_free(dev);
        }
        close(fd);
    }
    return ""; // non trouvé
}

Controller::Controller(QObject* parent)
: QObject(parent)
{
}

void Controller::initController(const std::string& expectedName)
{
    std::string device = findControllerDevice(expectedName);
    static bool messageAlreadyAppeared = false;
    if (device.empty()) {
        if ( !messageAlreadyAppeared){
            messageAlreadyAppeared = true;
            std::cerr << "Manette \"" << expectedName << "\" non trouvée !\n";
        }
        return;
    }
    fd = open(device.c_str(), O_RDONLY | O_NONBLOCK);

    if (fd < 0) {
        perror("open");
        return;
    }

    if (libevdev_new_from_fd(fd, &dev) < 0) {
        std::cerr << "Erreur init libevdev\n";
        return;
    }

    std::cout << "Manette détectée: "
    << libevdev_get_name(dev)
    << std::endl;
    isControllerFound = true;
    notifier = new QSocketNotifier(fd, QSocketNotifier::Read, this);

    connect(notifier, &QSocketNotifier::activated,
            this, &Controller::readController);

}

void Controller::readController()
{
    input_event ev;
    int rc;
    while ((rc = libevdev_next_event(dev,
        LIBEVDEV_READ_FLAG_NORMAL,
        &ev)) != -EAGAIN)
    {

        // Déconnexion détectée
        if (rc == -ENODEV) {
            std::cerr << "Manette déconnectée !\n";

            isControllerFound = false;

            if (notifier) {
                notifier->setEnabled(false);
                delete notifier;
                notifier = nullptr;
            }

            if (dev) {
                libevdev_free(dev);
                dev = nullptr;
            }

            if (fd >= 0) {
                close(fd);
                fd = -1;
            }

            return;
        }

        if (rc != 0)
            continue;

        if (ev.type == EV_KEY)
        {
            switch(ev.code)
            {
                case CODE_BUTTON_A: ButtonA = ev.value; break;
                case CODE_BUTTON_B: ButtonB = ev.value; break;
                case CODE_BUTTON_Y: ButtonY = ev.value; break;
                case CODE_BUTTON_X: ButtonX = ev.value; break;
                case CODE_BUTTON_XBOX: ButtonXBOX = ev.value; break;
            }
        }

        if (ev.type == EV_ABS)
        {
            switch(ev.code)
            {
                case CODE_THROTTLE_AXIS:
                    throttleAxis = remapInt(-ev.value, -32768, 32767, -1000, 1000);
                    throttleAxis = (abs(throttleAxis) * throttleAxis) / 15000;
                    break;

                case CODE_CW_ROTATION_TRIGGER:
                    CWRotationTrigger = remapInt(ev.value, 0, 1023, 0, JOYSTICK_RESOLUTION);
                    break;

                case CODE_CCW_ROTATION_TRIGGER:
                    CCWRotationTrigger = remapInt(ev.value, 0, 1023, 0, JOYSTICK_RESOLUTION);
                    break;

                case CODE_PITCH_AXIS:
                    pitchAxis = remapInt(-ev.value, -32768, 32767, -JOYSTICK_RESOLUTION, JOYSTICK_RESOLUTION);
                    break;

                case CODE_ROLL_AXIS:
                    rollAxis = remapInt(ev.value, -32768, 32767, -JOYSTICK_RESOLUTION, JOYSTICK_RESOLUTION);
                    break;
            }
        }
    }
}
