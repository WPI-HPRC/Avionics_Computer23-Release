#include <Arduino.h>
#include <Barometer/Barometer_SB.h>
#include <Temperature/TMP117.h>
#include <Flash/Flash.h>


TMP117 sensor; // Initalize sensor
FlashChip flash = FlashChip();
String structString = "";


void setup()
{
  Wire.begin();
  Serial.begin(9600);    // Start serial communication at 9600 baud
  Wire.setClock(400000);   // Set clock speed to be the fastest for better communication (fast mode)

  while (!Serial){}

  Serial.println("TMP117 Example 1: Basic Readings");
  if (sensor.begin() == true) // Function to check if the sensor will correctly self-identify with the proper Device ID/Address
  {
    Serial.println("Begin");
  }
  else
  {
    Serial.println("Device failed to setup- Freezing code.");
    while (1); // Runs forever
  }

  flash.init();


}

void loop()
{
  // Data Ready is a flag for the conversion modes - in continous conversion the dataReady flag should always be high
  if (sensor.dataReady() == true) // Function to make sure that there is data ready to be printed, only prints temperature values when data is ready
  {
    float tempF = sensor.readTempF();
    // Print temperature in °C and °F
    Serial.println(); // Create a white space for easier viewing
    Serial.print("Temperature in Fahrenheit: ");
    Serial.println(tempF);
    structString = String(tempF);
    flash.writeStruct(structString);
    delay(1000); // Read and log temperature every second
  }
}