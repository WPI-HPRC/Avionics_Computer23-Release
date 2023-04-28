#include <Arduino.h>
#include <lib/Flash/Flash.h>
#include <Wire.h>



FlashChip flash = FlashChip();
String structString = "";
uint32_t nextAddress;

int startingAddress = 0;


void setup() {
  
  Wire.begin();
  Serial.begin(115200); // Start serial communication at 115200 baud

  while (!Serial);

  Serial.println("Starting...");

  flash.init();
}

void loop() {
  
  if(Serial.available() > 1) {
    structString = Serial.readStringUntil('\n');
    flash.writeStruct(structString);
    Serial.println("Line Written to flash");
  }
}