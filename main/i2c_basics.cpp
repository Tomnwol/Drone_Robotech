
#include "i2c_basics.hpp"
// Déclaration des deux bus matériels
TwoWire I2C_MASTER_0_PORT = Wire;       // Bus I2C matériel 0
TwoWire I2C_MASTER_1_PORT = TwoWire(1); // Bus I2C matériel 1

//======================================================
// Écriture d’un octet sur un registre I2C
//======================================================
bool i2c_write_byte(TwoWire &bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    bus.beginTransmission(dev_addr);
    bus.write(reg_addr);
    bus.write(data);
    return bus.endTransmission() == 0; // true si succès
}

//======================================================
// Lecture de plusieurs octets sur un registre I2C
//======================================================
bool i2c_read_bytes(TwoWire &bus, uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
    bus.beginTransmission(dev_addr);
    bus.write(reg_addr);
    if (bus.endTransmission(false) != 0) return false; // restart condition
    bus.requestFrom(dev_addr, len);
    for (size_t i = 0; i < len && bus.available(); i++) {
        data[i] = bus.read();
    }
    return true;
}

//======================================================
// Initialisation I2C (bus, SDA, SCL et fréquence)
//======================================================
void i2c_master_init(TwoWire &bus, int sda_pin, int scl_pin, uint32_t clk_speed) {
    bus.begin(sda_pin, scl_pin);
    bus.setClock(clk_speed);
}
