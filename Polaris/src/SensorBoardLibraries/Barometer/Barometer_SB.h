#include <Arduino.h>
#include <Wire.h>
#include <SensorBoardLibraries\Sensor.h>

struct MS5611_Calibration_Data{
    uint16_t C1;
    uint16_t C2;
    uint16_t C3;
    uint16_t C4;
    uint16_t C5;
    uint16_t C6;
};


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
    void calculatePressureAndTemperature(uint32_t D1, uint32_t D2, float *Pressure, float *Temperature){
        int32_t dT = D2 - (uint32_t)calibrationData.C5 * (uint32_t)pow(2,8);
        int32_t TEMP = 2000 + (int32_t)dT * (int32_t)calibrationData.C6 / (int32_t)pow(2,23);
        int64_t OFF = (int64_t)calibrationData.C2 * (int64_t)pow(2,16) + (int64_t)dT * (int64_t)calibrationData.C4 / (int64_t)pow(2,7);
        int64_t SENS = (int64_t)calibrationData.C1 * (int64_t)pow(2,15) + (int64_t)dT * (int64_t)calibrationData.C3 / (int64_t)pow(2,8);
        int32_t P = (int32_t)((D1 * SENS / (int64_t)pow(2,21) - OFF) / (int64_t)pow(2,15));
        *Pressure = (float)P / 100.0;
        *Temperature = (float)TEMP / 100.0;
    };
    void calculatePressure(uint32_t D1, uint32_t D2, float *Pressure){
        int32_t dT = D2 - (uint32_t)calibrationData.C5 * (uint32_t)pow(2,8);
        int64_t OFF = (int64_t)calibrationData.C2 * (int64_t)pow(2,16) + (int64_t)dT * (int64_t)calibrationData.C4 / (int64_t)pow(2,7);
        int64_t SENS = (int64_t)calibrationData.C1 * (int64_t)pow(2,15) + (int64_t)dT * (int64_t)calibrationData.C3 / (int64_t)pow(2,8);
        int32_t P = (int32_t)((D1 * SENS / (int64_t)pow(2,21) - OFF) / (int64_t)pow(2,15));
        *Pressure = (float)P / 100.0;
    };

    // float calculateTemperature(uint32_t D2){
    //     // Serial.println("D2: " + String(D2));
    //     int32_t dT = D2 - (uint32_t)calibrationData.C5 * (uint32_t)pow(2,8);
    //     // Serial.println("dT: " + String(dT));
    //     int32_t TEMP = 2000 + (int32_t)dT * (int32_t)calibrationData.C6 / (int32_t)pow(2,23);
    //     // Serial.println("TEMP: " + String(TEMP));
    //     return (float)TEMP / 100.0;
    // }
    // float calculatePressure(uint32_t D1, uint32_t D2){
    //     // Serial.println("D1: " + String(D1));
    //     // Serial.println("D2: " + String(D2));
    //     int32_t dT = D2 - (uint32_t)calibrationData.C5 * (uint32_t)pow(2,8);
    //     // Serial.println("dT: " + String(dT));
    //     int64_t OFF = (int64_t)calibrationData.C2 * (int64_t)pow(2,16) + (int64_t)dT * (int64_t)calibrationData.C4 / (int64_t)pow(2,7);
    //     // Serial.println("OFF: " + String(OFF));
    //     int64_t SENS = (int64_t)calibrationData.C1 * (int64_t)pow(2,15) + (int64_t)dT * (int64_t)calibrationData.C3 / (int64_t)pow(2,8);
    //     // Serial.println("SENS: " + String(SENS));
    //     int32_t P = (int32_t)((D1 * SENS / (int64_t)pow(2,21) - OFF) / (int64_t)pow(2,15));
    //     // Serial.println("P: " + String(P));
    //     return (float)P / 100.0;
    // }
    static uint32_t processHighMidLowByte(uint8_t high, uint8_t mid, uint8_t low){
        return (uint32_t)high << 16 | (uint32_t)mid << 8 | (uint32_t)low;
    }


    private:
    int I2C_Address;
    TwoWire *I2C_BUS;
    void sendCommand(uint8_t command);
    MS5611_Calibration_Data calibrationData = {0,0,0,0,0,0};

};