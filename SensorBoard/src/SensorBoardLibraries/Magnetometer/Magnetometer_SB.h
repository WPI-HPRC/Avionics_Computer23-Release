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
const float A11 = .1249;
const float A12 = -.0059;
const float A13 = -.0002;
const float A21 = -.0059;
const float A22 = .1252;
const float A23 = -.0017;
const float A31 = -.0002;
const float A32 = .0017;
const float A33 = .1343;

const uint32_t B1_ = 128670;
const uint32_t B2_ = 130090;
const uint32_t B3_ = 131600;


class MMC5983MA : public Sensor{
    public:
    MMC5983MA(TwoWire &bus,uint8_t address);
    bool setup();
    void readSensor(uint8_t *Data, int StartIndex);
    void calculateCalibratedValues(uint32_t Raw_X, uint32_t Raw_Y, uint32_t Raw_Z, float *Calibrated_X, float *Calibrated_Y, float *Calibrated_Z);
    
    static uint32_t process18BitResolution(uint8_t HighByte, uint8_t LowByte, uint8_t ExtraByte, uint8_t Axis);
    private:
    bool CheckProductID();
    void writeRegister(uint8_t SubAddress, uint8_t data);
    void readRegister(uint8_t SubAddress, int length, uint8_t *data);
    void sensorEnable();
    int I2C_Address;
    TwoWire *I2C_BUS;
};