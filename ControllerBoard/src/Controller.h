#ifndef Controller_h
#define Controller_h

#include <Arduino.h>

class Controller
{
    public:
        Controller();
        uint8_t calcAbPct(float alt, int8_t velZ, int8_t velLat);
        float targetDragCoefficient();

    private:
};

#endif