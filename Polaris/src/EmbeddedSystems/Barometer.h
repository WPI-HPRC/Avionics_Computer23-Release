#include <Arduino.h>
#include <EmbeddedSystems\Structs.h>
#include <Wire.h>

// Commands for the barometer
const uint8_t RESET = 0x1E;
const uint8_t ADC_READ = 0x00;
const uint8_t PROM_Data_Address0 = 0xA0;
const uint8_t PROM_Data_Address1 = 0xA2;
const uint8_t PROM_Data_Address2 = 0xA4;
const uint8_t PROM_Data_Address3 = 0xA6;
const uint8_t PROM_Data_Address4 = 0xA8;
const uint8_t PROM_Data_Address5 = 0xAA;
const uint8_t PROM_Data_Address6 = 0xAC;
const uint8_t PROM_Data_Address7 = 0xAE;

// D1 OSR - Uncompensated pressure reading oversampling ratio
const uint8_t D1_OSR_256 = 0x40;
const uint8_t D1_OSR_512 = 0x42;
const uint8_t D1_OSR_1024 = 0x44;
const uint8_t D1_OSR_2048 = 0x46;
const uint8_t D1_OSR_4096 = 0x48;

// D2 OSR - Uncompensated temperature reading oversampling ratio
const uint8_t D2_OSR_256 = 0x50;
const uint8_t D2_OSR_512 = 0x52;
const uint8_t D2_OSR_1024 = 0x54;
const uint8_t D2_OSR_2048 = 0x56;
const uint8_t D2_OSR_4096 = 0x58;

class Barometer
{
    public:
    ms5611CalibrationData calibrationData;
    uint8_t Buffer[3]; //Buffer for reading data from the barometer
    TwoWire *I2C_BUS; // I2C bus that the barometer is connected to
    uint8_t I2C_Address; // I2C address of the barometer
    uint32_t RawTemperatureData; // Raw temperature data from the barometer
    uint32_t RawPressureData; // Raw pressure data from the barometer
    float Temperature; // Temperature in degrees celcius
    float CompensatedPressure; // Pressure in mbar
    Barometer(TwoWire &bus,uint8_t address);
    bool setup();
    void ReadCalibrationData();
    void sendCommand(uint8_t command);
    void reset();
    void readRawTemperature();
    void readRawPressure();
    void CollectRawData();
    void ProcessRawData();
    void readSensor();

    float getCalPressure();
    float getCalTemperature();
};

