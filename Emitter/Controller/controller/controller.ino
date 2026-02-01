/*Radio*/
#include <stdint.h>
#include <SPI.h>
#include <RF24.h>

#define pinCE   7             // On associe la broche "CE" du NRF24L01 à la sortie digitale D7 de l'arduino
#define pinCSN  10             // On associe la broche "CSN" du NRF24L01 à la sortie digitale D8 de l'arduino
const byte pipeAddress[5] = {'P','I','P','E','1'};

RF24 radio(pinCE, pinCSN);

struct QT_Payload {
  uint16_t throttle;
  uint8_t killSwitch;
  uint8_t failSafeSwitch;
  uint16_t KP;
  uint16_t KI;
};

QT_Payload qt_payload;

#define QT_PAYLOAD_SIZE 8 // OCTETS
void setupNRF() {
  if (!radio.begin()) {
    Serial.println("Erreur: NRF24 non détecté !"); // else : allumer une led idéalement
    while (1);
    return;
  }
  // Paramètres identiques à l'émetteur
  radio.setAutoAck(false);   // DÉSACTIVE LES ACK
  radio.setRetries(0, 0);
  radio.setChannel(0);              // Même canal que l'Arduino
  radio.setPALevel(RF24_PA_LOW);    // Même puissance
  radio.setDataRate(RF24_1MBPS);    // Même débit
  radio.openReadingPipe(0, pipeAddress); // Même adresse que l'émetteur
  radio.stopListening();              // Arrêt de l'écoute du NRF24 (signifiant qu'on va émettre, et non recevoir, ici)
}

void sendDataNRF(){
  //Envoi radio
  #define NRF_PAYLOAD_SIZE 32 // 32 MAX
  unsigned char NRF_Payload[NRF_PAYLOAD_SIZE];
  uint16_t JoyThrottle = qt_payload.throttle; //analogRead(A0);
  uint16_t JoyYaw = 0;
  uint16_t JoyRoll = 0;
  uint16_t JoyPitch = 0;
  uint16_t GP_Pot = 0;
  uint8_t SWFailSafe = qt_payload.failSafeSwitch;
  uint8_t SWKillSwitch = qt_payload.killSwitch;
  uint16_t KP = qt_payload.KP;
  uint16_t KI = qt_payload.KI;
  /*Remplissage du payload*/
  NRF_Payload[0] = JoyThrottle & 255; // 255 = 0x11111111 (8 bits = sizeof(char))
  NRF_Payload[1] = JoyThrottle >> 8; // reste
  NRF_Payload[2] = JoyYaw & 255;
  NRF_Payload[3] = (JoyYaw>>8);
  NRF_Payload[4] = JoyRoll & 255;
  NRF_Payload[5] = (JoyRoll>>8);
  NRF_Payload[6] = JoyPitch & 255;
  NRF_Payload[7] = (JoyPitch>>8);
  NRF_Payload[8] = GP_Pot & 255;
  NRF_Payload[9] = (GP_Pot>>8);
  NRF_Payload[10] = SWFailSafe;
  NRF_Payload[11] = SWKillSwitch;
  NRF_Payload[12] = KP & 255; //ATTENTION KP ET KI *100, il faudra float puis les diviser dans le drone
  NRF_Payload[13] = (KP>>8);
  NRF_Payload[14] = KI & 255;
  NRF_Payload[15] = (KI>>8);

  radio.write(&NRF_Payload, sizeof(NRF_Payload));
  delay(5);
}

void setup() {
  Serial.begin(115200);
  setupNRF();
}

void loop() {
  static uint8_t buffer[QT_PAYLOAD_SIZE+1];
  static uint8_t index = 0;

  while (Serial.available()) {
    uint8_t byteIn = Serial.read();

    // Synchronisation sur '\n'
    if (byteIn == '\n') {
      if (index == QT_PAYLOAD_SIZE) {  // on a reçu exactement QT_PAYLOAD_SIZE données avant \n, sinon reset
        /*Réception des données de l'interface QT*/
        qt_payload.throttle       = (buffer[1] << 8) | buffer[0];
        qt_payload.killSwitch     = buffer[2];
        qt_payload.failSafeSwitch = buffer[3];
        qt_payload.KP             = (buffer[5] << 8) | buffer[4];
        qt_payload.KI             = (buffer[7] << 8) | buffer[6];

        // ACK debug
        Serial.print("ACK throttle=");
        Serial.print(qt_payload.throttle);
        Serial.print(" KI=");
        Serial.print(qt_payload.KI);
        Serial.print(" killSwitch=");
        Serial.print(qt_payload.killSwitch);
        Serial.print(" FS=");
        Serial.println(qt_payload.failSafeSwitch);
        sendDataNRF();
      }
      index = 0; // reset pour la prochaine trame
    }
    else {
      if (index < QT_PAYLOAD_SIZE) {
        buffer[index++] = byteIn;
      } else {
        // overflow => reset
        index = 0;
      }
    }
  }
}
