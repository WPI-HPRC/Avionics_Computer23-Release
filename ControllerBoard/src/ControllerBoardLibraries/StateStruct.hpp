#include <Arduino.h>
struct StateStruct
{
    float vel_vert;   // Vertical velocity. Given in m/s. 
    float vel_lat;    // Lateral velocity. Given in m/s. 
    float vel_total;  // Total velocity. Given in m/s. 
};