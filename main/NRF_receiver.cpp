#include <SPI.h>
#include <RF24.h>
#include "NRF_receiver.hpp"
// Définition des pins SPI pour ESP32
#define CE_PIN   4
#define CSN_PIN  5

//Pin non précisés : 
//SCK -> 18
//MISO -> 19
//MOSI -> 23
// Création de l'objet RF24
RF24 radio(CE_PIN, CSN_PIN);
// Adresse du pipe d'émission de l'Arduino
const byte pipeAddress[5] = {'P','I','P','E','1'};

#define PAYLOAD_SIZE 32
char payload[PAYLOAD_SIZE];

unsigned long radio_delta_time = 0; /*Sert à activer le Failsafe mode en cas de non-communication prolongée*/
unsigned long radio_last_time = 0;

uint16_t joyThrottle = 10; //Armer le drone  
uint16_t joyYaw = 0;
uint16_t joyRoll = 0;
uint16_t joyPitch = 0;
uint16_t GP_Pot = 0;
uint8_t SWFailSafe = 0;
uint8_t SWKillSwitch = 0;
float QT_KP = 0;
float QT_KI = 0;
float QT_KD = 0;
uint8_t offsetMotorFL = 0;
uint8_t offsetMotorFR = 0;
uint8_t offsetMotorBL = 0;
uint8_t offsetMotorBR = 0;
void setupNRF() {
  if (!radio.begin()) {
    Serial.println("Erreur: NRF24 non détecté !"); // else : allumer une led idéalement
    while (1);
    return;
  }
  // Paramètres identiques à l'émetteur
  radio.setAutoAck(false);   // DÉSACTIVE LES ACK
  radio.setRetries(0, 0);    // ⬅️ aucune tentative de retry
  radio.setChannel(0);              // Même canal que l'Arduino
  radio.setPALevel(RF24_PA_LOW);    // Même puissance
  radio.setDataRate(RF24_1MBPS);    // Même débit
  radio.openReadingPipe(0, pipeAddress); // Même adresse que l'émetteur
  radio.startListening();           // On passe en mode réception

  //Serial.println("NRF24 prêt à recevoir !");
  radio_last_time = millis();
}


void readNRFData() {
  radio_delta_time = millis() - radio_last_time;
  if (radio.available()) {
    radio.read(&payload, PAYLOAD_SIZE);  // Lecture du message
    if(payload[0] != NULL){
      radio_last_time = millis();
      //Serial.print("Message reçu: ");
      joyThrottle = payload[0] + (payload[1]<<8);
      joyYaw = payload[2] + (payload[3]<<8);
      joyRoll = payload[4] + (payload[5]<<8);
      joyPitch = payload[6] + (payload[7]<<8);
      GP_Pot = payload[8] + (payload[9]<<8);
      SWFailSafe = payload[10];
      SWKillSwitch = payload[11];
      QT_KP = ((float)(uint16_t)(payload[12] + (payload[13]<<8))/100.0); // Récupération du KP et KI, en pensant bien à diviser 100 pour retrouver les 2 décimales
      QT_KI = ((float)(uint16_t)(payload[14] + (payload[15]<<8))/100.0);
      QT_KD = ((float)(uint16_t)(payload[16] + (payload[17]<<8))/100.0);
      offsetMotorFL = payload[18];
      offsetMotorFR = payload[19];
      offsetMotorBL = payload[20];
      offsetMotorBR = payload[21];
    }
    //Serial.print("Throttle : ");
    //Serial.println(joyThrottle);
  }
}
