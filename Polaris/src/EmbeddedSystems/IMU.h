#include <Arduino.h>
#include <EmbeddedSystems\Structs.h>

#include <Wire.h>
//class for interfacing with the ICM-46288-P
enum RateGyroRange
{
    GYRO_15_625DPS,
    GYRO_31_25DPS,
    GYRO_62_5DPS,
    GYRO_125DPS,
    GYRO_250DPS,
    GYRO_500DPS,
    GYRO_1000DPS,
    GYRO_2000DPS
};
enum AccelRange
{
    ACCEL_2G,
    ACCEL_4G,
    ACCEL_8G,
    ACCEL_16G
};

class IMU{
    public:
    int I2C_Address; // I2C address of the IMU
    TwoWire *I2C_BUS; // I2C bus that the IMU is connected to
    uint8_t Buffer[12]={}; // Buffer to store the data from the IMU
    Vector3<int16_t> Gyro_Raw_Values; // raw values from the gyroscope in LSBs
    Vector3<int16_t> Accelerometer_Raw_Values; // raw values from the accelerometer in LSBs
    AccelRange Accelerometer_Range; // range of the accelerometer
    RateGyroRange RateGyro_Range; // range of the gyroscope
    IMU(TwoWire &bus,uint8_t address); // constructor for the IMU class
    void readSensor(); 
    void setup();
    void WriteToRegister(uint8_t SubAddress, uint8_t data);
    void ReadRegister(uint8_t SubAddress, uint8_t length, uint8_t *data);
    bool CheckWhoAmI();
    bool SensorEnable();

    Vector3<int16_t> getAccelVals();
};