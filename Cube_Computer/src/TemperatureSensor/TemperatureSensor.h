#include <Arduino.h>
#include <Wire.h>
#include <Sensor.h>
/*
    @author Samay Govani
    @brief TMP117 Temperature Sensor Class for the Sensor Board
    Worcester Polytechnic Institute High Power Rocketry Club
*/

class TMP117 : public Sensor{
    public:
    TMP117(TwoWire &bus,uint8_t address);
    bool setup();
    void readSensor(uint8_t *Data, int StartIndex);
    const uint16_t TEMP_MOST_RECENT_VALUE = 0x00;

    private:
    int I2C_Address;
    TwoWire *I2C_BUS;
    const float TEMP_LSB = 0.0078125; // 0.0078125 degrees C per LSB
    bool isDataReady();
};