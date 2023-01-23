#include <Arduino.h>
#include <Wire.h>
#include <Sensor.h>

/*
    @author Samay Govani
    @brief MS5611 Barometer Class for the Sensor Board
    Worcester Polytechnic Institute High Power Rocketry Club
*/
class MS5611 : public Sensor{
    public:
    MS5611(TwoWire &bus,uint8_t address);
    bool setup();
    void readSensor(uint8_t *Data, int StartIndex);
    void readCalibrationData();

    private:
    int I2C_Address;
    TwoWire *I2C_BUS;
    void sendCommand(uint8_t command);
};