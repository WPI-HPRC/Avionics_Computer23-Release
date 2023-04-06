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

  telemBoard->setState(RX);
  telemBoard->init();
}

void loop() {
    if(timer.check() == 1) {
        counter++;
        timestamp = counter * (CONVERSION/LOOP_FREQUENCY);

        if(telemBoard->getState() == TX) {
          TelemetryPacket txPacket;
          txPacket.timestamp = timestamp;
          txPacket.state = 0;
          txPacket.abPct = 0;
          txPacket.altitude = 100;
          txPacket.acX = 10;
          txPacket.acY = 10;
          txPacket.acZ = 10;
          txPacket.gyX = 10;
          txPacket.gyY = 10;
          txPacket.gyZ = 10;
          telemBoard->setPacket(txPacket);
        }

        telemBoard->onLoop(timestamp);
        timer.reset();
    }
} 