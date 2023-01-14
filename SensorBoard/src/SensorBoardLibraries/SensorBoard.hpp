#include <SensorBoardLibraries\IMU\IMU_SB.h>
#include <SensorBoardLibraries\Magnetometer\Magnetometer_SB.h>
#include <SensorBoardLibraries\Barometer\Barometer_SB.h>
#include <ACAN2517FD.h>
#include "Config.h"

/*
    @author Samay Govani
    @brief Sensorboard class contains all the sensor objects and functions to read data from them and store it in a buffer
    @details Worcester Polytechnic Institute High Power Rocketry Club
*/
class Sensorboard{
    private:
    MS5611 barometer = MS5611(Wire, BARO_I2C_ADDRESS); // Barometer
    ICM42688P imu = ICM42688P(Wire, IMU_I2C_ADDRESS); // IMU
    MMC5983MA mag = MMC5983MA(Wire, MAG_I2C_ADDRESS); // Magnetometer
    uint8_t Buffer[29] = {0};
    public:
    Sensorboard(){};

    /*
        @brief Sets up all the sensors
        @details Returns true if all sensors are setup correctly
    */
    bool setup(){
        if (!imu.setup()) return false;
        if (!barometer.setup()) return false;
        if (!mag.setup()) return false;
        return true;
    }
    
    /*
        @brief Reads data from all sensors and stores it in the buffer
        @details The buffer is a 29 byte array that contains the data from the sensors in the following order:
        IMU: 12 bytes Accelerometer: 6 bytes (H byte, L Byte for each axis) Gyroscope: 6 bytes (H byte, L Byte for each axis) in X,Y,Z order
        Barometer: 6 bytes Pressure: 3 bytes Temperature: 3 bytes
        Magnetometer: 7 bytes X: 2 bytes Y: 2 bytes Z: 2 bytes Extra Byte for 18 Bit Resolution for each axis
        Time: 4 bytes milliseconds since the program started
    */
    void readSensor(){
        // Store sensor data in buffer
        imu.readSensor(Buffer,0);
        barometer.readSensor(Buffer,12);
        mag.readSensor(Buffer,18);
        // Store the time in the last 4 bytes of the buffer
        uint32_t time = millis();
        Buffer[25] = (uint8_t)((time >> 24) & 0xFFU);
        Buffer[26] = (uint8_t)((time >> 16) & 0xFFU);
        Buffer[27] = (uint8_t)((time >> 8) & 0xFFU);
        Buffer[28] = (uint8_t)(time & 0xFFU);
    }

    /*
        @brief Returns the buffer at the specified index
        @param index The index of the buffer to return
        @return The byte at the specified index if the index is valid (0-28) otherwise returns 0
    */
    uint8_t getBuffer(int index){
        if (index < 0 || index > 28) return 0;
        return Buffer[index];
    }
};