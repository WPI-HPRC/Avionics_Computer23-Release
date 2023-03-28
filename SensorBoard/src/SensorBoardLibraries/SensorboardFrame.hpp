#include <Arduino.h>
struct SensorboardFrame
{
    float X_accel;  // Acceleration in m/s^2
    float Y_accel; // Acceleration in m/s^2
    float Z_accel; // Acceleration in m/s^2
    float X_gyro; // Angular velocity in degrees/s
    float Y_gyro; // Angular velocity in degrees/s
    float Z_gyro; // Angular velocity in degrees/s
    float X_mag; // 
    float Y_mag; //
    float Z_mag; // 
    float Pressure; // Pressure in mBar
    float Latitude; // Latitude in degrees
    float Longitude; // Longitude in degrees
    float N_Velocity; // North Velocity in m/s
    float E_Velocity; // East Velocity in m/s
    float D_Velocity; // Down Velocity in m/s
    uint32_t time; // Time in milliseconds since the start of the program
};

