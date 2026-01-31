/*Radio*/
#include <stdint.h>
#include <SPI.h>
#include <RF24.h>

#define pinCE   7             // On associe la broche "CE" du NRF24L01 à la sortie digitale D7 de l'arduino
#define pinCSN  10             // On associe la broche "CSN" du NRF24L01 à la sortie digitale D8 de l'arduino
const byte pipeAddress[5] = {'P','I','P','E','1'};     

RF24 radio(pinCE, pinCSN); 

void loop() {
  uint16_t filtered_value  = uint16_t(FilteredRead(A0, &filtA0, lutLX)); // Penser à modifier
  uint8_t KillSwitch = digitalRead(4);
  uint8_t FailSafeMode = digitalRead(5);
  static int i = 0;
  //Envoi radio
  #define PAYLOAD_LEN 32 // 32 MAX
  unsigned char payload[PAYLOAD_LEN]; 
  uint16_t JoyThrottle = filtered_value; //analogRead(A0);
  uint16_t JoyYaw = 0;
  uint16_t JoyRoll = 0;
  uint16_t JoyPitch = 0;
  uint16_t GP_Pot = 0;
  uint8_t SWFailSafe = FailSafeMode;
  uint8_t SWKillSwitch = KillSwitch;
  /*Remplissage du payload*/
  payload[0] = JoyThrottle & 255; // 255 = 0x11111111 (8 bits = sizeof(char))
  payload[1] = JoyThrottle >> 8; // reste
  payload[2] = JoyYaw & 255;
  payload[3] = (JoyYaw>>8);
  payload[4] = JoyRoll & 255;
  payload[5] = (JoyRoll>>8);
  payload[6] = JoyPitch & 255;
  payload[7] = (JoyPitch>>8);
  payload[8] = GP_Pot & 255;
  payload[9] = (GP_Pot>>8);
  payload[10] = SWFailSafe;
  payload[11] = SWKillSwitch;

  radio.write(&payload, sizeof(payload)); 
  delay(5);
}

/********************
** Détails techniques **

Temps de montée (0 -> 2000) : 300ms
Temps de descente (2000 -> 0) : 400ms

Zone joystick inutilisée : 1200 / 2048 [~60%]

Overshoot : 44%
********************/

