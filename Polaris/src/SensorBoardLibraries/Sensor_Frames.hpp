#include <Arduino.h>

struct SensorFrame
{
    float ac_x;  // Acceleration in g
    float ac_y; // Acceleration in g
    float ac_z; // Acceleration in g
    float gy_x; // Angular velocity in degrees/s
    float gy_y; // Angular velocity in degrees/s
    float gy_z; // Angular velocity in degrees/s 
    uint32_t X_mag;
    uint32_t Y_mag;
    uint32_t Z_mag;
    float Pressure; // Pressure in mBar
    float Temperature; // Temperature in degrees C
};

struct GPSFrame
{
    float N_Position; // North Position in Meters
    float E_Position; // East Position in Meters
    float D_Position; // Down Position in Meters
    float N_Velocity; // North Velocity in Meters/Second
    float E_Velocity; // East Velocity in Meters/Second
    float D_Velocity; // Down Velocity in Meters/Second
};