#ifndef __UDP_RECEPTION_H
#define __UDP_RECEPTION_H
#include <Arduino.h>
#include "types.hpp"
extern uint16_t joyThrottle; //analogRead(A0);
extern int16_t joyYaw; // - 1000 to 1000
extern int16_t joyRoll;
extern int16_t joyPitch;
extern uint16_t GP_Pot;
extern uint8_t SWFailSafe;
extern uint8_t SWKillSwitch;
extern float QT_KP;
extern float QT_KI;
extern float QT_KD;
extern uint8_t offsetMotorFL;
extern uint8_t offsetMotorFR;
extern uint8_t offsetMotorBL;
extern uint8_t offsetMotorBR;

extern unsigned long udp_last_time;
extern unsigned long udp_delta_time;
#define BATTERY_PIN 34
#define BATTERY_MIN 1910 // mV correspondant à 0%
#define BATTERY_MAX 2320 // mV correspondant à 100%

void handleSendTelemetry(Euler att_telemetry, uint16_t MOT_FL_telemetry, uint16_t MOT_FR_telemetry, uint16_t MOT_BR_telemetry, uint16_t MOT_BL_telemetry);
void setupUDP();
void readUDPData();


#endif
