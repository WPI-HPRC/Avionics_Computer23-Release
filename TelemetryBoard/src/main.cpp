#include <Arduino.h>

#include <GroundStation/TelemetryBoard.h>
#include <lib/MyMetroTimer.h>

#define CONVERSION 1000
#define LOOP_FREQUENCY 10

MyMetro timer = MyMetro(CONVERSION / LOOP_FREQUENCY);

int counter = 0;
uint32_t timestamp;

TelemetryBoard * telemBoard;

void setup() {
  Serial.begin(57600);

  telemBoard = new TelemetryBoard();

  telemBoard->setState(TX);
  telemBoard->init();
}

void loop() {
    if(timer.check() == 1) {
        counter++;

        timestamp = counter * (CONVERSION/LOOP_FREQUENCY);

        telemBoard->onLoop(timestamp);
        timer.reset();
    }
} 