#include "ServoDriver.h"

Servo myServo;

ServoDriver::ServoDriver(uint8_t servoPin) {
    myServo.attach(servoPin);
}

void ServoDriver::init() {
    setPosition(0);
    delay(1000);
}

void ServoDriver::setPosition(uint8_t percent) {
    uint8_t position = map(percent, 0, 100, AIRBRAKES_LOWER_BOUND, AIRBRAKES_UPPER_BOUND);
    myServo.write(position);
}