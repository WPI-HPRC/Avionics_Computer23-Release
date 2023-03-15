#include <Arduino.h>
struct SensorboardFrame
{
    float X_accel;  // Acceleration in m/s^2
    float Y_accel; // Acceleration in m/s^2
    float Z_accel; // Acceleration in m/s^2
    float X_gyro; // Angular velocity in degrees/s
    float Y_gyro; // Angular velocity in degrees/s
    float Z_gyro; // Angular velocity in degrees/s
    float X_mag; // Magnetic field in Gauss
    float Y_mag; // Magnetic field in Gauss
    float Z_mag; // Magnetic field in Gauss
    float Pressure; // Pressure in mBar
    float Temperature; // Temperature in degrees Celsius
    float latitude; // Latitude in degrees
    float longitude; // Longitude in degrees
    uint32_t time; // Time in milliseconds since the start of the program
};

