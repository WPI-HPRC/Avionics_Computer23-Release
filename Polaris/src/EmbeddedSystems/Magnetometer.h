#include <Arduino.h>
#include <EmbeddedSystems\Structs.h>
#include <Wire.h>

class Magnetometer{
    public:
    int I2C_Address;
    TwoWire *I2C_BUS;
    Vector3<int32_t> Magnetometer_Raw_Values;
    Vector3<float> Magnetometer_Calibrated_Values;
    Vector3<int32_t> Magnetometer_HardIron_Values;
    Vector3<float> Magnetometer_SoftIron_Values;
    Magnetometer(TwoWire &bus,uint8_t address){
        I2C_BUS = &bus;
        I2C_Address = address;
    }
    uint8_t buffer[7];
    void WriteToRegister(uint8_t SubAddress, uint8_t data);
    void WriteToRegister(uint8_t SubAddress, uint8_t data, uint32_t TimeToDelayAfterWrite);
    void ReadRegister(uint8_t SubAddress, uint8_t length, uint8_t *data);
    void readSensor();
    bool CheckWhoAmI();
    void setup();
    bool SensorEnable();
    void reset();
};