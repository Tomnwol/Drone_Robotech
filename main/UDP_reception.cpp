#include "UDP_reception.hpp"
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "ESP32_Tom";
const char* password = "12345678";

WiFiUDP udp;
const int udpPort = 4210;

uint8_t payload[22]; // comme NRF_Payload

unsigned long udp_last_time = 0;
unsigned long udp_delta_time = 0;

uint16_t joyThrottle = 0; //analogRead(A0);
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

//Emmit data from ESP to PC
BATTERY_PIN = GPIO_NUM_36;
IPAddress pcIP;
uint16_t pcPort = 0;

void setupUDP() {
    WiFi.softAP(ssid, password);
    Serial.print("IP ESP32 : ");
    Serial.println(WiFi.softAPIP());

    udp.begin(udpPort);
    Serial.println("UDP en écoute...");
    udp_last_time = millis();
}

uint16_t readBatteryMV() {
    int raw = analogRead(BATTERY_PIN);

    // Converti ADC (0-4095) en mV (supposons 3.3V référence)
    float voltage = (raw / 4095.0) * 3300; // en mV
    return (uint16_t)voltage;
}

uint8_t batteryPercentage() {
    uint16_t mv = readBatteryMV();

    // Contrainte entre min/max
    if (mv >= BATTERY_MAX) return 100;
    if (mv <= BATTERY_MIN) return 0;

    // Map 1910–2320 mV → 0–100 %
    float perc = ((float)(mv - BATTERY_MIN)) / (BATTERY_MAX - BATTERY_MIN) * 100.0;
    return (uint8_t)perc;
}

void handleSendTelemetry(){
    static unsigned long lastSend = 0;
    if (millis() - lastSend > 500) {
        lastSend = millis();
        uint8_t value = batteryPercentage(); // Lis la valeur de la batterie ( entre 2,32V et 1,91)
        sendTelemetry(value);
    }
}

void sendTelemetry(uint8_t batteryVoltage) {
    if (pcPort != 0){
        udp.beginPacket(pcIP, pcPort);
        udp.write(batteryVoltage, 1);
        udp.endPacket();
    }
}




void readUDPData() {
    udp_delta_time = millis() - udp_last_time;
    int packetSize = udp.parsePacket();
    if (packetSize == 22) { // on attend 22 octets
        udp_last_time = millis();
        udp.read(payload, 22);
        pcIP = udp.remoteIP();
        pcPort = udp.remotePort();

        joyThrottle = payload[0] + (payload[1]<<8);
        joyYaw      = payload[2] + (payload[3]<<8);
        joyRoll     = payload[4] + (payload[5]<<8);
        joyPitch    = payload[6] + (payload[7]<<8);
        GP_Pot      = payload[8] + (payload[9]<<8);
        SWFailSafe  = payload[10];
        SWKillSwitch= payload[11];
        QT_KP       = ((float)(uint16_t)(payload[12] + (payload[13]<<8))/100.0);
        QT_KI       = ((float)(uint16_t)(payload[14] + (payload[15]<<8))/100.0);
        QT_KD       = ((float)(uint16_t)(payload[16] + (payload[17]<<8))/100.0);
        offsetMotorFL = payload[18];
        offsetMotorFR = payload[19];
        offsetMotorBL = payload[20];
        offsetMotorBR = payload[21];

        // Debug
        /*Serial.print("Throttle : "); Serial.println(joyThrottle);
        Serial.print("Yaw : "); Serial.println(joyYaw);
        Serial.print("Roll : "); Serial.println(joyRoll);
        Serial.print("Pitch : "); Serial.println(joyPitch);
        Serial.print("QT_KI : "); Serial.println(QT_KI);*/
    }
}
