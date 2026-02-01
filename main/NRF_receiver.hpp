#ifndef __NRF_RECEIVER_H
#define __NRF_RECEIVER_H

extern uint16_t joyThrottle; //analogRead(A0);
extern uint16_t joyYaw;
extern uint16_t joyRoll;
extern uint16_t joyPitch;
extern uint16_t GP_Pot;
extern uint8_t SWFailSafe;
extern uint8_t SWKillSwitch;
extern uint8_t SWKillSwitch;
extern float QT_KP;
extern float QT_KI;

extern unsigned long radio_last_time;
extern unsigned long radio_delta_time;
void setupNRF();
void readNRFData();


#endif
