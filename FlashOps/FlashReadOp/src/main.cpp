#include <Arduino.h>
#include <SPIFlash.h>
#include <Wire.h>



SPIFlash flash = SPIFlash(7); // 7 for Cube Board, 10 for Polaris
uint32_t nextAddress;
String structString = "";
String addressString = "";
String read = "";


void setup()
{
  Wire.begin();
  Serial.begin(9600); 

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

  delay(1000);


  // All of this in setup should be removed later when reading 

  flash.readStr(0, read);
  Serial.println(read);


}

void loop()
{

  // ====================== Reading Flash ======================
  // nextAddress += 256; // This comes first cuz the first page is for the address 
  // String data = "";

  // Serial.print("Data from Flash:  "); // For Polaris
  // flash.readStr(nextAddress, data, true);
  // Serial.println(data);

  // delay(10); 

  // ====================== Testing Flash ======================

  flash.readStr(0, read); 
  Serial.println(read);

  nextAddress = (uint32_t)(read.toInt());
  Serial.print("Next Address: "); 
  Serial.println(nextAddress);

  nextAddress += 256;
  addressString = String(nextAddress);
  flash.eraseSection(0, 256);
  flash.writeStr(0, addressString);
  flash.readStr(0, read);
  Serial.print("Address String: ");
  Serial.println(read);
  Serial.println(" ");

  delay(1000);
}
