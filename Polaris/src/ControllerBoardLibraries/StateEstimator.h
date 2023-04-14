#include <Arduino.h>

struct StateStruct{
    float vel_vert = 0; // vertical velocity
    float vel_lat = 0; // lateral velocity
    float vel_total = 0; // total velocity
};

class StateEstimator {
    private:
        const float timeStep = 0.01;
        float prev_avg_alt = 0;
        float vel_x;
        float vel_y;
        float vel_z;
        float altitudeBuffer[10];
        uint8_t BuffInd = 0;

    public:
        StateStruct state;

        StateStruct getState(float alt, float ac_x, float ac_y, float ac_z);

};