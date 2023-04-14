#ifndef Controller_h
#define Controller_h

#include <Arduino.h>
#include "../lib/MetroTimer/Metro.h"

class Controller
{
    public:
        Controller();
        uint8_t calcAbPct(float alt, int8_t velLat, int8_t velY);
        float targetDragCoefficient(float alt, int8_t velLat, int8_t velY, int8_t dt, float P, float altZero, float tZero);
        float rk4(float alt, int8_t velLat, int8_t velY, int8_t dt, float currCD, float P, float altZero, float tZero);
        float calcLatAcc(float velLat, float v, float currCD, float rho, float A);
        float calcVertAcc(float velY, float v, float currCD, float rho, float A);
        float density(float alt, float tZero, float p);
        float temperature_at_altitude(float tZero, float alt);
        float pressure_at_altitude(float P, float h, float T);
        void setInitPressureTemp(float initP, float initT);

    private:
        float length = 1.472; // [in] length of the airbrakes
        float allowableError = 0.0001; // TODO: find out what this should be
        float targetApogee = 3625; // [ft] TODO: change for diff launches
        float area = 3.14159 * 0.078359 * 0.078359; // cross-sectional area
        float g = 9.8;
        float m = 17.28; // dry mass of the rocket
        float dt = 0.001; // 10 milliseconds
        float R = 8.314; 
        float M = 0.02896;
        float l = -6.5/1000;

        float ext = 0; // current airbrakes extension
        float cd = 0; // current coeff of drag

        float pZero;
        float tZero;
};

#endif