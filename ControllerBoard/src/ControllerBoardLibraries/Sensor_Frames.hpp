#include <Arduino.h>
struct SensorFrame
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
    float Temperature; // Temperature in degrees C
    uint32_t time; // Time in milliseconds since the start of the program
};

struct GPSFrame
{
    float N_Position; // North Position in Meters
    float E_Position; // East Position in Meters
    float D_Position; // Down Position in Meters
    float N_Velocity; // North Velocity in Meters/Second
    float E_Velocity; // East Velocity in Meters/Second
    float D_Velocity; // Down Velocity in Meters/Second
    uint32_t time; // Time in milliseconds since the start of the program
};