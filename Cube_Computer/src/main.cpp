#include <Arduino.h>
#include <Temperature/TMP117.h>
#include <SPIFlash.h>
#include <Adafruit_AHTX0.h>
#include <MS5x.h>

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

  Serial.println("Starting...");

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

  if (flash.eraseChip())
  {
    Serial.println("Chip Erased");
  }
  else
  {
    Serial.println("Chip Erase Failed");
  }
}

void loop()
{
  barometer.checkUpdates(); // apparently this will keep the barometer from delaying code execution

  if (tmp.dataReady() == true && barometer.isReady()) // Check if the sensors have new data
  {
    tempF = tmp.readTempF();
    aht.getEvent(&humidity, &temp); 
    pressure = barometer.GetPres();

    Serial.print("Temperature, Pressure, Humidity from sensors: ");
    Serial.print(tempF);
    Serial.print(", ");
    Serial.print(pressure);
    Serial.print(", ");
    Serial.println(humidity.relative_humidity);

    structString = String(tempF) + "," + String(pressure) + "," + String(humidity.relative_humidity);
    nextAddress += 256;
    flash.writeStr(nextAddress, structString, true);
    Serial.print("Temperature, Pressure, Humidity Written to Flash: ");
    Serial.println(structString);

    delay(100); // Run at 10Hz
  }
}
