#ifndef ServoDriver_h
#define ServoDriver_h

#include <Arduino.h>
#include <servo.h>

class ServoDriver
{
public:
    ServoDriver(uint8_t servoPin);
    void init();
    void setPosition(uint8_t percent);

private:
    uint8_t servoPin;
    const uint8_t AIRBRAKES_LOWER_BOUND = 142;
    const uint8_t AIRBRAKES_UPPER_BOUND = 175;

};

#endif