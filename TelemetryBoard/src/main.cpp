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

void setup() {
  Serial.begin(115200);

  telemBoard.initTelem();
  telemBoard.setState(RX); //Default state

}

void loop() {

  

  if(millis() - prevTime >= loopFrequency) {
    prevTime = millis();
    telemBoard.onLoop();
  }

}