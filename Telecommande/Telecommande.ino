/*
   ______               _                  _///  _           _                   _
  /   _  \             (_) |  __\| | | |                 (_) |  [_| |__  ___  ___ _  ___  _ __ | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___ |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \ | | | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__ |\__,_|\___| | |
                                                                                      \_|
  Fichier:      HelloWorldNRF24L01-Emetteur
  Description:  Emission d'un "Hello World" via un NRF24L01
  Auteur:       Passion-Électronique

  Librairie utilisée : https://github.com/nRF24/RF24

  Créé le 19.03.2021
*/
#include <SPI.h>
#include <RF24.h>

#define pinCE   7             // On associe la broche "CE" du NRF24L01 à la sortie digitale D7 de l'arduino
#define pinCSN  8             // On associe la broche "CSN" du NRF24L01 à la sortie digitale D8 de l'arduino
#define tunnel  "PIPE1"       // On définit un "nom de tunnel" (5 caractères), pour pouvoir communiquer d'un NRF24 à l'autre

RF24 radio(pinCE, pinCSN);    // Instanciation du NRF24L01

const byte adresse[6] = tunnel;               // Mise au format "byte array" du nom du tunnel
char message[] = "";     // Message à transmettre à l'autre NRF24 (32 caractères maxi, avec cette librairie)

void setup() {
  radio.begin();                      // Initialisation du module NRF24
  radio.openWritingPipe(adresse);     // Ouverture du tunnel en ÉCRITURE, avec le "nom" qu'on lui a donné
  radio.setPALevel(RF24_PA_HIGH);      // Sélection d'un niveau "MINIMAL" pour communiquer (pas besoin d'une forte puissance, pour nos essais)
  radio.setDataRate(RF24_1MBPS);
  radio.stopListening();              // Arrêt de l'écoute du NRF24 (signifiant qu'on va émettre, et non recevoir, ici)
}

/*
  Récupération des valeurs x et y:
  x_value = message[0] + message[1]<<8;
  y_value = message[2] + message[3]<<8;
*/

void loop() {
  uint16_t x_value = analogRead(A0);
  message[0] = x_value & 255; // 255 = 0x11111111 (8 bits = sizeof(char))
  message[1] = x_value>>8; // reste

  uint16_t y_value = analogRead(A1);
  message[2] = y_value & 255;
  message[3] = y_value>>8;
  radio.write(&message, sizeof(message));     // Envoi de notre message
  delay(100);                                // … toutes les secondes !
}