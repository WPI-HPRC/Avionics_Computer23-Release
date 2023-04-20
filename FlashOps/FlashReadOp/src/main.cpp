#include <Arduino.h>
#include <SPIFlash.h>
#include <Wire.h>



SPIFlash flash = SPIFlash(7); // 7 for Cube Board, 10 for Polaris
uint32_t nextAddress;
String structString = "";
// String addressString = "";
// String read = "";


void setup()
{
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

  // flash.readStr(0, read); 
  // Serial.println(read);

  // nextAddress = (uint32_t)(read.toInt());
  // Serial.print("Next Address: "); 
  // Serial.println(nextAddress);

  // // change the address at the beginning of the flash 5 times 
  // for (int i = 0; i < 5; i++)
  // {
  //   nextAddress += 256;
  //   addressString = String(nextAddress);
  //   flash.eraseSection(0, 256);
  //   flash.writeStr(0, addressString);
  //   Serial.print("Next Address: "); 
  //   flash.readStr(0, read); 
  //   Serial.println(read);
  //   delay(1000);
  // }

  // Serial.println(nextAddress);

  // flash.readStr(0, read); 
  // Serial.println(read);

}

void loop()
{
    nextAddress += 256;
    String data = "";

    Serial.print("Data from Flash:  "); // For Polaris
    flash.readStr(nextAddress, data, true);
    Serial.println(data);

    delay(10); 

}
