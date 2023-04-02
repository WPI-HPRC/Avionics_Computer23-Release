#include <Wire.h>
#include <Arduino.h>

class MAX_M10S_I2C{
    public:
    private:
        uint8_t I2C_Address;
        TwoWire *I2C_BUS;
};