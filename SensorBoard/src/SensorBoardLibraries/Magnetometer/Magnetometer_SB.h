#include <Arduino.h>
#include <Wire.h>
#include <SensorBoardLibraries\Sensor.h>

/*
    @author Samay Govani
    @brief MMC5983MA Magnetometer Class for the Sensor Board
    Worcester Polytechnic Institute High Power Rocketry Club
*/

class MMC5983MA : public Sensor{
    public:
    MMC5983MA(TwoWire &bus,uint8_t address);
    bool setup();
    void readSensor(uint8_t *Data, int StartIndex);

    private:
    bool CheckProductID();
    void writeRegister(uint8_t SubAddress, uint8_t data);
    void readRegister(uint8_t SubAddress, int length, uint8_t *data);
    void sensorEnable();
    int I2C_Address;
    TwoWire *I2C_BUS;
};