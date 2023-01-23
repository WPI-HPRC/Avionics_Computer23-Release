#pragma once
#include  <Arduino.h>
/*
    @class Abstract class for sensors
*/
class Sensor{
    public:
    virtual bool setup() = 0;
    virtual void readSensor(uint8_t *Data,int StartIndex) = 0;
};