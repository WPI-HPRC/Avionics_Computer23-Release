#include <Arduino.h>
#include <Wire.h>
#include <SensorBoardLibraries/Config.h>
#include <SensorBoardLibraries/Sensor.h>

/*
    @author Samay Govani
    @brief ICM-4288P IMU Class for the Sensor Board
    Worcester Polytechnic Institute High Power Rocketry Club
*/

class ICM42688P : public Sensor{

    public:
    void readSensor(uint8_t *Data, int StartIndex);
    bool setup();
    ICM42688P(TwoWire &bus, uint8_t address);
    static int16_t processHighLowByte(uint8_t high, uint8_t low);
    static float processAxis(int16_t axisReading, float sensitivity){
        return (float)axisReading / (float)sensitivity;
    }

    private:
    uint8_t I2C_Address;
    TwoWire *I2C_BUS;
    bool writeRegister(uint8_t reg, uint8_t data, uint8_t bank); 
    void writeRegister(uint8_t reg, uint8_t data);
    void readRegister(uint8_t reg, uint8_t length, uint8_t* data);
    void readRegister(uint8_t reg, uint8_t length, uint8_t* data, uint8_t bank);
    bool switchToBank(uint8_t bank);
    bool checkWhoAmI();
    bool sensorEnable();
    bool setFilters();
    bool setODR_FS();
    
    
};