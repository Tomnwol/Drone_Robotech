#include <stdio.h>
#include <stdint.h>

int main(){

    unsigned char message[32];
    uint16_t JoyThrottle = 256;
    uint16_t JoyYaw = 2084;
    uint16_t JoyRoll = 178;
    uint16_t JoyPitch = 36;
    uint16_t GP_Pot = 704;
    uint8_t SWFailSafe = 0;
    uint8_t SWKillSwitch = 1;

    message[0] = JoyThrottle & 255; // 255 = 0x11111111 (8 bits = sizeof(char))
    message[1] = JoyThrottle >> 8; // reste
    message[2] = JoyYaw & 255;
    message[3] = (JoyYaw>>8);
    message[4] = JoyRoll & 255;
    message[5] = (JoyRoll>>8);
    message[6] = JoyPitch & 255;
    message[7] = (JoyPitch>>8);
    message[8] = GP_Pot & 255;
    message[9] = (GP_Pot>>8);
    message[10] = SWFailSafe;
    message[11] = SWKillSwitch;



    JoyThrottle = message[0] + (message[1]<<8);
    JoyYaw = message[2] + (message[3]<<8);
    JoyRoll = message[4] + (message[5]<<8);
    JoyPitch = message[6] + (message[7]<<8);
    GP_Pot = message[8] + (message[9]<<8);
    SWFailSafe = message[10];
    SWKillSwitch = message[11];
    printf("Valeurs recuperees : \n%d\n%d\n%d\n%d\n%d\n%d\n%d\n", JoyThrottle, JoyYaw, JoyRoll, JoyPitch, GP_Pot, SWFailSafe, SWKillSwitch);
    //radio.write(&message, sizeof(message));     // Envoi de notre message

    return 0;
}
