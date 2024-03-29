#include <Arduino.h>
#include <Temperature/TMP117.h>
#include <SPIFlash.h>
#include <Adafruit_AHTX0.h>
#include <MS5x.h>
#include <GroundStation/Telemetry.h>

#include <./lib/Metro/Metro.h>
#include "Config.h"

Metro timer = Metro(1000 / 10);
int counter = 0;
uint32_t timestamp = 0;

TMP117 tmp; // Initalize sensor
Adafruit_AHTX0 aht;
MS5x barometer(&Wire);
SPIFlash flash = SPIFlash(7);
sensors_event_t humidity, temp;
uint32_t nextAddress = 0;
String structString = "";
double pressure = 0;
double tempF = 0;

void setup()
{
  Wire.begin();
  Serial.begin(9600); // Start serial communication at 9600 baud

  SPI.begin();

  Serial.println(" ");

  while(barometer.connect()>0) { // barometer.connect starts wire and attempts to connect to sensor
    Serial.println(F("Error connecting to barometer..."));
  	delay(500);
  }
  Serial.println(F("Barometer Found"));
  delay(5);

  if (tmp.begin() == true) // Function to check if the sensor will correctly self-identify with the proper Device ID/Address
  {
    Serial.println("TMP117 Found");
  }
  else
  {
    Serial.println("Device failed to setup- Freezing code.");
    while (1)
    {
    }
  }
  if (!aht.begin())
  {
    Serial.println("Could not find AHT? Check wiring");
    while (1)
      delay(10);
  }
  Serial.println("AHT20 Found");

  if (flash.begin())
  {
    Serial.println("Flash Chip Found");
  }

  Serial.print("Flash Capacity: ");
  Serial.println(flash.getCapacity());
  Serial.print("Max Page: ");
  Serial.println(flash.getMaxPage());
  Serial.println("[Flash Chip]: Initialization Complete");

}

void loop()
{

  if(timer.check() == 1) {
    timestamp = counter * (1000/10);

    barometer.checkUpdates(); // apparently this will keep the barometer from delaying code execution
    tempF = tmp.readTempF();
    aht.getEvent(&humidity, &temp);
    pressure = barometer.GetPres();

    structString = String(timestamp) + "," + String(tempF) + "," + String(pressure) + "," + String(humidity.relative_humidity);
    flash.writeStr(nextAddress, structString, true);
    nextAddress += 256;


    Serial.println(structString);

    counter++;
    timer.reset();
  }
}