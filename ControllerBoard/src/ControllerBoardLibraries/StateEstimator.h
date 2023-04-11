#include <Arduino.h>

struct StateStruct{
    float vel_vert = 0; // vertical velocity
    float vel_lat = 0; // lateral velocity
    float vel_total = 0; // total velocity
};

class StateEstimator {
    private:
    public:

        const float timeStep = 0.01;
        float prevAltitude = 0;
        StateStruct state;

        StateStruct getState(float alt, float ac_X, float ac_Y, float ac_Z);

};