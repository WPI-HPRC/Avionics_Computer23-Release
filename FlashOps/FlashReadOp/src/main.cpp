#include <Arduino.h>
#include <SPIFlash.h>
#include <Wire.h>

SPIFlash flash = SPIFlash(7); // 7 for Cube Board, 10 for Polaris
uint32_t nextAddress = 16777216 * 0.875;
String structString = "";
String addressString = "";
String read = "";
boolean done = false;
String data = "";


void setup()
{
  Wire.begin();
  Serial.begin(9600); // 115200 for Polaris, 9600 for Cube Board

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
}

void loop()
{

  // ====================== Reading Flash ======================

  while (nextAddress < 16777216 ){
    flash.readStr(nextAddress, data, true);
    Serial.println(data);

    nextAddress += 256; 
  }


}
