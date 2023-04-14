#include <Arduino.h>
#include <Temperature/TMP117.h>
#include <SPIFlash.h>
#include <Adafruit_AHTX0.h>

// #include <MS5x.h>

// #include <Flash/Flash.h>

TMP117 tmp; // Initalize sensor
Adafruit_AHTX0 aht;
// MS5x barometer(&Wire);
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
  // Wire.setClock(400000); // Set clock speed to be the fastest for better communication (fast mode)

  while (!Serial)
  {
  }

  Serial.println("TMP117 Example 1: Basic Readings");
  if (tmp.begin() == true) // Function to check if the sensor will correctly self-identify with the proper Device ID/Address
  {
    Serial.println("Begin");
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
  Serial.println("AHT10 or AHT20 found");

  // while(barometer.connect()>0) { // barometer.connect starts wire and attempts to connect to sensor
  // 	Serial.println(F("Error connecting to barometer..."));
  // 	delay(500);
  // }
  // Serial.println(F("Connected to barometer"));
  // delay(5);

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
  // barometer.checkUpdates();

  if (tmp.dataReady() == true) // Function to make sure that there is data ready to be printed, only prints temperature values when data is ready
  {
    tempF = tmp.readTempF();
    aht.getEvent(&humidity, &temp); 
    // pressure = barometer.GetPres();

    Serial.print("Temperature, Pressure, Humidity from sensors: ");
    Serial.print(tempF);
    Serial.print(", ");
    Serial.print(pressure);
    Serial.print(", ");
    Serial.println(humidity.relative_humidity);

    String data = "";
    structString = String(tempF) + "," + String(pressure) + "," + String(humidity.relative_humidity);
    nextAddress += 256;
    flash.writeStr(nextAddress, structString, true);
    Serial.print("Temperature, Pressure, Humidity Written to Flash: ");
    Serial.println(structString);

    flash.readStr(nextAddress, data, true);
    Serial.print("Temperature, Pressure, Humidity Read from Flash:  ");
    Serial.println(data);

    delay(1000); // Read and log temperature every second
  }
}
