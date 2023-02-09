#include <Arduino.h>

#include <GroundStation/TelemetryBoard.h>

TelemetryBoard * telemBoard = new TelemetryBoard(teensy);

RocketPacket currentRocketPacket;

void setup() {
  Serial.begin(115200);

  telemBoard->init();
  telemBoard->setState(TX);
}

void loop() {

    if(telemBoard->getState() == TX) {
      currentRocketPacket.timestamp = millis();
      currentRocketPacket.state = 10;
      currentRocketPacket.altitude = 100.0;
      currentRocketPacket.pressure = 29.92;
      currentRocketPacket.temperature = 5.0;

      telemBoard->setCurrentPacket(currentRocketPacket);
    }

    telemBoard->onLoop();

    delay(1000);

} 