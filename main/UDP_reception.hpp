#ifndef __UDP_RECEPTION_H
#define __UDP_RECEPTION_H
#include <Arduino.h>

extern uint16_t joyThrottle; //analogRead(A0);
extern uint16_t joyYaw;
extern uint16_t joyRoll;
extern uint16_t joyPitch;
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
void setupUDP();
void readUDPData();


#endif
