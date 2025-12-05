#ifndef __NRF_RECEIVER_H
#define __NRF_RECEIVER_H

extern uint16_t joyThrottle; //analogRead(A0);
extern uint16_t joyYaw;
extern uint16_t joyRoll;
extern uint16_t joyPitch;
extern uint16_t GP_Pot;
extern uint8_t SWFailSafe;
extern uint8_t SWKillSwitch;

void setupNRF();
void readNRFData();


#endif