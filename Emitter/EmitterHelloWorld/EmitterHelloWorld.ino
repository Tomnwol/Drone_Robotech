#include <SPI.h>
#include <RF24.h>

//Arduino Controller pinout
#define pinCE   4             // On associe la broche "CE" du NRF24L01 à la sortie digitale D7 de l'arduino
#define pinCSN  5             // On associe la broche "CSN" du NRF24L01 à la sortie digitale D8 de l'arduino

RF24 radio(pinCE, pinCSN);    // Instanciation du NRF24L01

const byte pipeAddress[5] = {'P','I','P','E','1'};             // Mise au format "byte array" du nom du tunnel
const char message[] = "Hello World !!!";     // Message à transmettre à l'autre NRF24 (32 caractères maxi, avec cette librairie)

void setup() {
  pinMode(pinCSN, OUTPUT);
  Serial.begin(115200);
  if (!radio.begin()) {
    Serial.println("NRF non detecte !");
    radio.printDetails();
    while(1);
  }
  radio.setChannel(70);
  radio.openWritingPipe(pipeAddress);     // Ouverture du tunnel en ÉCRITURE, avec le "nom" qu'on lui a donné
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);      // Sélection d'un niveau "MINIMAL" pour communiquer (pas besoin d'une forte puissance, pour nos essais)
  radio.stopListening();              // Arrêt de l'écoute du NRF24 (signifiant qu'on va émettre, et non recevoir, ici)
  
   delay(1000);

}

void loop() {
  bool ok = radio.write(&message, sizeof(message));

 Serial.println(ok ? "ACK OK" : "ACK FAIL");     // Envoi de notre message
 Serial.println("Message sent");
 //radio.printDetails();
 delay(3000);                                // … toutes les secondes !
}
/*
#include <SPI.h>
#include <RF24.h>

//ESP Drone pinout
#define CE_PIN   4
#define CSN_PIN  5        

RF24 radio(CE_PIN, CSN_PIN);    // Instanciation du NRF24L01

const byte pipeAddress[5] = {'P','I','P','E','1'};
char message[32];                     // Avec cette librairie, on est "limité" à 32 caractères par message
int debug_msg = 0;
void setup() {
  // Initialisation du port série (pour afficher les infos reçues, sur le "Moniteur Série" de l'IDE Arduino)
  Serial.begin(115200);
  delay(1000);
  Serial.println("Récepteur NRF24L01");
  Serial.println("");

  // Partie NRF24
  if (!radio.begin()) {
    Serial.println("NRF non detecte !");
    radio.printDetails();
    while(1);
  }
  radio.setChannel(0);
  radio.openReadingPipe(0, pipeAddress);  // Ouverture du tunnel en LECTURE, avec le "nom" qu'on lui a donné
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);      // Sélection d'un niveau "MINIMAL" pour communiquer (pas besoin d'une forte puissance, pour nos essais)
  radio.startListening();             // Démarrage de l'écoute du NRF24 (signifiant qu'on va recevoir, et non émettre quoi que ce soit, ici)
}

void loop() {
  debug_msg ++;
  if (debug_msg >= 500000){
    debug_msg = 0;
    Serial.println("debug");
    radio.printDetails();
  }
  // On vérifie à chaque boucle si un message est arrivé
  if (radio.available()) {
    radio.read(&message, sizeof(message));                        // Si un message vient d'arriver, on le charge dans la variable "message"
    Serial.print("Message reçu : "); Serial.println(message);     // … et on l'affiche sur le port série !
  }
}*/