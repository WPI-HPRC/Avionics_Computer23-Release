#include <Arduino.h>
#include <SPIFlash.h>
#include <Wire.h>



SPIFlash flash = SPIFlash(7); // 7 for Cube Board, 10 for Polaris
uint32_t nextAddress;


void setup() {
  
  Wire.begin();
  Serial.begin(9600); // Start serial communication at 9600 baud

  while (!Serial)
  {
  }

  Serial.println("Starting...");

  if (flash.begin())
  {
    Serial.println("Flash Chip Found");
  }

  Serial.print("Flash Capacity: ");
  Serial.println(flash.getCapacity());
  Serial.print("Max Page: ");
  Serial.println(flash.getMaxPage());
  Serial.println("[Flash Chip]: Initialization Complete");
  
  if (flash.eraseChip())
  {
    Serial.println("Chip Erased");
  }
  else
  {
    Serial.println("Chip Erase Failed");
  }

  // Things for testing the remembering address method 

  String data = String(256);
  String read = "";

  flash.writeStr(0, data, true); // write 256 to address 0
  flash.readStr(0, read); 
  Serial.println(read);

}

void loop() {
  // do nothing
}