#include <Arduino.h>
struct TelemetryFrame
{
    uint32_t timestamp; // System time from power board, running from startup onward. Given in milliseconds.
    uint8_t state;      // Rocket mission state from state machine.
    float altitude;     // Converted altitude measurement (TODO: altitude estimate from EKF). Given in meters.
    int8_t temperature; // Temperature measurement. (TODO: Implement this and figure out what units)
    uint8_t vBatt;      // Battery voltage from power board. Given in Volts. Scaled by 20.
    uint8_t abPct;      // Airbrake actuation level. Given as percentage from 0 to 100.
    int16_t ac_x;       // X-axis acceleration. Given in m/s^2. Scaled by 100.
    int16_t ac_y;       // Y-axis acceleration. Given in m/s^2. Scaled by 100.
    int16_t ac_z;       // Z-axis acceleration. Given in m/s^2. Scaled by 100.
    int16_t gy_x;       // Angular rotation rate about X-axis. Given in degrees/s. Scaled by 10.
    int16_t gy_y;       // Angular rotation rate about Y-axis. Given in degrees/s. Scaled by 10.
    int16_t gy_z;       // Angular rotation rate about Z-axis. Given in degrees/s. Scaled by 10.
};