#include <SensorBoardLibraries\IMU\IMU_SB.h>
#include <SensorBoardLibraries\Magnetometer\Magnetometer_SB.h>
#include <SensorBoardLibraries\Barometer\Barometer_SB.h>
#include "Config.h"
#include "SensorboardFrame.hpp"

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
    SensorboardFrame frame;
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
        this->ProcessBuffer();
        Serial.println("Read Sensor");
    }

    void ProcessBuffer(){
        // Process the buffer and store the data in the frame, once the frame is updated set newFrame to true

        // Accelerometer
        frame.X_accel = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[0],Buffer[1]), 2048.0);
        frame.Y_accel = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[2],Buffer[3]), 2048.0);
        frame.Z_accel = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[4],Buffer[5]), 2048.0);


        // Gyroscope
        frame.X_gyro = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[6],Buffer[7]), 16.4);
        frame.Y_gyro = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[8],Buffer[9]), 16.4);
        frame.Z_gyro = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[10],Buffer[11]), 16.4);

        // Barometer
        barometer.calculatePressureAndTemperature(MS5611::processHighMidLowByte(Buffer[12],Buffer[13],Buffer[14]),MS5611::processHighMidLowByte(Buffer[15],Buffer[16],Buffer[17]),&frame.Pressure,&frame.Temperature);

        // Magnetometer
        mag.calculateCalibratedValues(MMC5983MA::process18BitResolution(Buffer[18],Buffer[19],Buffer[25],0),MMC5983MA::process18BitResolution(Buffer[20],Buffer[21],Buffer[25],1),MMC5983MA::process18BitResolution(Buffer[22],Buffer[23],Buffer[25],2),&frame.X_mag,&frame.Y_mag,&frame.Z_mag);

        frame.time = millis();

        frame.latitude = 0;
        frame.longitude = 0;
    }
};