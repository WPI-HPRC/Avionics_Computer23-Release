#include <Arduino.h>
#include <SPIFlash.h>
#include <Wire.h>

SPIFlash flash = SPIFlash(10); // 7 for Cube Board, 10 for Polaris
uint32_t nextAddress = 256;
String structString = "";
String addressString = "";
String read = "";
boolean done = false;

void setup()
{
  Wire.begin();
  Serial.begin(115200);

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

  nextAddress += 256; 
  String data = "";

  // Serial.print("Data from Flash:  "); 
  flash.readStr(nextAddress, data, true);
  Serial.println(data);

  delay(10);

}
