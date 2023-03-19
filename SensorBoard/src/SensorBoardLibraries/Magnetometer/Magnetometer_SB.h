#include <Arduino.h>
#include <Wire.h>
#include <SensorBoardLibraries\Sensor.h>

/*
    @author Samay Govani
    @brief MMC5983MA Magnetometer Class for the Sensor Board
    Worcester Polytechnic Institute High Power Rocketry Club
*/

/*
Calibration Data 
*/
// #define A11 .1249
// #define A12 .-.0059
// #define A13 .-.0002
// #define A21 .-.0059
// #define A22 .1252
// #define A23 .-.0017
// #define A31 .-.0002
// #define A32 . .0017
// #define A33 .1343

// #define B1 128670
// #define B2 130090
// #define B3 131600


class MMC5983MA : public Sensor{
    public:
    MMC5983MA(TwoWire &bus,uint8_t address);
    bool setup();
    void readSensor(uint8_t *Data, int StartIndex);
    /*
    */
    
    private:
    bool CheckProductID();
    void writeRegister(uint8_t SubAddress, uint8_t data);
    void readRegister(uint8_t SubAddress, int length, uint8_t *data);
    void sensorEnable();
    int I2C_Address;
    TwoWire *I2C_BUS;
};