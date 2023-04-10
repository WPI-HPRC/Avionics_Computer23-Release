#ifndef Controller_h
#define Controller_h

#include <Arduino.h>

class Controller
{
    public:
        Controller();
        uint8_t Controller::calcAbPct(float alt, int8_t velLat, int8_t velY);
        float Controller::targetDragCoefficient(float alt, int8_t velLat, int8_t velY, int8_t dt, float P, float altZero, float tZero);
        float Controller::rk4(float alt, int8_t velLat, int8_t velY, int8_t dt, float currCD, float P, float altZero, float tZero);
        float Controller::calcLatAcc(float velLat, float v, float currCD, float rho, float A);
        float Controller::calcVertAcc(float velY, float v, float currCD, float rho, float A);
        float Controller::density(float alt, float tZero, float p);
        float Controller::temperature_at_altitude(float tZero, float alt);
        float Controller::pressure_at_altitude(float P, float h, float T);
        float Controller::setInitPressureTemp(float initP, float initT);

    private:
        float length = 1.472; // length of the airbrakes
        float allowableError = 0.1; // TODO: find out what this should be
        float targetApogee = 5000; // TODO: change for diff launches
        float area = 3.14159 * 0.078359 * 0.078359; // cross-sectional area
        float g = 9.8;
        float m = 24.829; // mass of the rocket
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