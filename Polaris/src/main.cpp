#include <Arduino.h>

#include <GroundStation/TelemetryBoard.h>
#include <lib/MyMetroTimer.h>

#define CONVERSION 1000
#define LOOP_FREQUENCY 10

MyMetro timer = MyMetro(CONVERSION / LOOP_FREQUENCY);

int counter = 0;
uint32_t timestamp;

TelemetryBoard telemBoard = TelemetryBoard();

void setup() {
  Serial.begin(115200);

  telemBoard.setState(TX);
  telemBoard.init();
}

void loop() {
    if(timer.check() == 1) {
        counter++;

        timestamp = counter * (CONVERSION/LOOP_FREQUENCY);

        telemBoard.onLoop(timestamp);
        timer.reset();

    }
} 