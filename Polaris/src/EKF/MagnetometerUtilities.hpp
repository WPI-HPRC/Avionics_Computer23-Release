#pragma once

#include <Arduino.h>

struct MagFieldVector {
    double declination;
    double inclination;
    double intensity;
};

class MagnetometerUtilities {
    public:
        MagnetometerUtilities() {};

        float calcMagDipAngle(double );

    private:



};