#pragma once
#include <Arduino.h>


struct quaternion {
    float w, x, y, z; // w is the real part and the rest are the imaginary parts
};

template <typename T> struct Vector3 {
    T x, y, z;
    T getMagnitude(){
        return sqrt(x*x + y*y + z*z);
    }
};

struct ms5611CalibrationData{
    uint16_t SensT1,OFFT1,TCS,TCO,Tref,TEMPSENS = 0;
};

