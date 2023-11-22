#include <SensorBoardLibraries\IMU\IMU_SB.h>
#include <SensorBoardLibraries\Barometer\Barometer_SB.h>
#include "Config.h"
#include "Sensor_Frames.hpp"
#include <lib/MMC5983/SparkFun_MMC5983MA_Arduino_Library.h>

/*
    @author Samay Govani
    @brief Sensorboard class contains all the sensor objects and functions to read data from them and store it in a buffer
    @details Worcester Polytechnic Institute High Power Rocketry Club
*/
class Sensorboard{
    private:
    MS5611 barometer = MS5611(Wire, BARO_I2C_ADDRESS); // Barometer
    ICM42688P imu = ICM42688P(Wire, IMU_I2C_ADDRESS); // IMU
    SFE_MMC5983MA mag;
    // SFE_UBLOX_GNSS gps; // GPS

    uint8_t Buffer[18] = {0};

    public:
    Sensorboard(){};
    SensorFrame Inertial_Baro_frame;
    GPSFrame gpsFrame;

    /*
        @brief Sets up all the sensors
        @details Returns true if all sensors are setup correctly
    */
    bool setup(){
        if (!imu.setup()) return false;
        if (!barometer.setup()) return false;
        if(!mag.begin()) return false;

        mag.softReset();
        // mag.setFilterBandwidth(100);
        // mag.setContinuousModeFrequency(50);
        // mag.enableAutomaticSetReset();
        // mag.enableContinuousMode();
        // if (!gps.begin(Wire)) return false;
        // gps.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
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
    void readInertialSensors(){
        // Store sensor data in buffer
        imu.readSensor(Buffer,0);
        barometer.readSensor(Buffer,12);
        this->ProcessBuffer();
    }

    void ProcessBuffer(){
        // Process the buffer and store the data in the frame, once the frame is updated set newFrame to true

        // Accelerometer
        Inertial_Baro_frame.ac_x = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[0],Buffer[1]), 2048.0);
        Inertial_Baro_frame.ac_y = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[2],Buffer[3]), 2048.0);
        Inertial_Baro_frame.ac_z = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[4],Buffer[5]), 2048.0);


        // Gyroscope
        Inertial_Baro_frame.gy_x = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[6],Buffer[7]), 16.4);
        Inertial_Baro_frame.gy_y = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[8],Buffer[9]), 16.4);
        Inertial_Baro_frame.gy_z = ICM42688P::processAxis(ICM42688P::processHighLowByte(Buffer[10],Buffer[11]), 16.4);

        Inertial_Baro_frame.X_mag = mag.getMeasurementX();
        Inertial_Baro_frame.Y_mag = mag.getMeasurementY();
        Inertial_Baro_frame.Z_mag = mag.getMeasurementZ();
        // Barometer
        barometer.calculatePressureAndTemperature(MS5611::processHighMidLowByte(Buffer[12],Buffer[13],Buffer[14]),MS5611::processHighMidLowByte(Buffer[15],Buffer[16],Buffer[17]),&Inertial_Baro_frame.Pressure,&Inertial_Baro_frame.Temperature);
    }
};