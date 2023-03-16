/**
 * @file main.cpp
 * @author Ground Station
 * @brief main ino for telemetry board
 * @version 0.1
 * @date 2023-1-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <Arduino.h>

#include <GroundStation/TelemetryBoard.h>

constexpr static uint32_t loopFrequency = 100UL;
volatile uint32_t prevTime = 0;

TelemetryBoard telemBoard = TelemetryBoard();

RocketPacket newRocketPacket;

void setup() {
  Serial.begin(115200);

  telemBoard.init();
  telemBoard.setState(TX);

}

void loop() {

  if(telemBoard.getState() == TX) {
      newRocketPacket.timestamp = millis();
      newRocketPacket.state = 0;
      newRocketPacket.temperature = 100.0;
      newRocketPacket.pressure = 29.92;
  }

  telemBoard.onLoop();

  delay(100);
}