#include <SPI.h>
#include <RF24.h>

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

// Taille du payload
#define PAYLOAD_SIZE 32

char payload[PAYLOAD_SIZE];

void setupNRF() {
  if (!radio.begin()) {
    Serial.println("Erreur: NRF24 non détecté !");
    //while (1);
  }
  // Paramètres identiques à l'émetteur
  radio.setChannel(0);              // Même canal que l'Arduino
  radio.setPALevel(RF24_PA_HIGH);    // Même puissance
  radio.setDataRate(RF24_1MBPS);    // Même débit
  radio.openReadingPipe(0, pipeAddress); // Même adresse que l'émetteur
  radio.startListening();           // On passe en mode réception

  Serial.println("NRF24 prêt à recevoir !");
}

void readNRFData() {
  if (radio.available()) {
    radio.read(&payload, PAYLOAD_SIZE);  // Lecture du message
    //Serial.print("Message reçu: ");
    //Serial.println(payload);

    uint16_t throttle = payload[0] + payload[1] << 8;
    uint16_t rotation = payload[2] + payload[3] << 8;
    //Serial.print("Throttle : ");
    //Serial.println(throttle);

    //Serial.print("Rotation : ");
    //Serial.println(rotation);
  }
}
