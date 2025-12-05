/*
 *  ______               _                  _///  _           _                   _
 * /   _  \             (_)                |  __\| |         | |                 (_)
 * |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
 * |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
 * |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
 * \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
 *                                                                                     | |
 *                                                                                     \_|
 * Fichier:      HelloWorldNRF24L01-Emetteur
 * Description:  Emission d'un "Hello World" via un NRF24L01
 * Auteur:       Passion-Électronique
 *
 * Librairie utilisée : https://github.com/nRF24/RF24
 *
 * Créé le 19.03.2021
 */
#include <SPI.h>
#include <RF24.h>

#define pinCE   7             // On associe la broche "CE" du NRF24L01 à la sortie digitale D7 de l'arduino
#define pinCSN  8             // On associe la broche "CSN" du NRF24L01 à la sortie digitale D8 de l'arduino
#define tunnel  "PIPE1"       // On définit un "nom de tunnel" (5 caractères), pour pouvoir communiquer d'un NRF24 à l'autre

RF24 radio(pinCE, pinCSN);    // Instanciation du NRF24L01

const byte pipeAddress[5] = {'P','I','P','E','1'};             // Mise au format "byte array" du nom du tunnel
//const char message[] = "Hello World !!!";     // Message à transmettre à l'autre NRF24 (32 caractères maxi, avec cette librairie)

void setup() {
    radio.begin();                      // Initialisation du module NRF24
    radio.setChannel(0);
    radio.openWritingPipe(pipeAddress);     // Ouverture du tunnel en ÉCRITURE, avec le "nom" qu'on lui a donné
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_HIGH);
    radio.stopListening();              // Arrêt de l'écoute du NRF24 (signifiant qu'on va émettre, et non recevoir, ici)
}

void loop() {
    unsigned char payload[32];
    /*Chaque message transmit via NRF24l01 fait forcéement 32 octets. On peut donc y stocker 32 uint8_t. Pour les uint16_t, il faudra décomposer le stockage en 2 octets */
    uint16_t JoyThrottle = 256; //analogRead(A0);
    uint16_t JoyYaw = 2084;
    uint16_t JoyRoll = 178;
    uint16_t JoyPitch = 36;
    uint16_t GP_Pot = 704;
    uint8_t SWFailSafe = 0;
    uint8_t SWKillSwitch = 1;
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

    radio.write(&message, sizeof(message));     // Envoi de notre message
    delay(1000);                                // … toutes les secondes !
}
