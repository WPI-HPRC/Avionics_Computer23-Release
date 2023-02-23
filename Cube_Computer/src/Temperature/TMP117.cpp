#include <Arduino.h>
#include <Wire.h>
#include "TMP117_Registers.h"
#include "TMP117.h"

/* CONSTRUCTOR
    This function will use the main I2C port on the Arduino 
	by default, but this is configurable with the setBus function.
	This needs to be called when running the example sketches to
	initialize the sensor and be able to call to the library. 
*/
TMP117::TMP117()
{
}

/* BEGIN
    This function checks if the TMP will ACK over I2C, and
	if the TMP will correctly self-identify with the proper
	device ID. This will set the address of the device along 
	with setting the wire for the I2C Communication. 
	This will return true if both checks pass.
*/
bool TMP117::begin(uint8_t sensorAddress, TwoWire &wirePort)
{
	_i2cPort = &wirePort;			// Chooses the wire port of the device
	_deviceAddress = sensorAddress; // Sets the address of the device

	//make sure the TMP will acknowledge over I2C
	_i2cPort->beginTransmission(_deviceAddress);
	if (_i2cPort->endTransmission() != 0)
	{
		return false;
	}

	uint16_t deviceID = readRegister(TMP117_DEVICE_ID); // reads registers into rawData

	//make sure the device ID reported by the TMP is correct
	//should always be 0x0117
	if (deviceID != DEVICE_ID_VALUE)
	{
		return false;
	}

	return true; //returns true if all the checks pass
}

/* GET ADDRESS 
	This function returns the address of the device once
	called upon. This address can only be 0x48 (GND), 
	0x49 (V+), 0x4A (SDA), and 0x4B (SCL)
*/
uint8_t TMP117::getAddress()
{
	return _deviceAddress;
}

/* READ REGISTER
	This function reads the register bytes from the sensor when called upon.
	This reads 2 bytes of information from the 16-bit registers. 
*/
uint16_t TMP117::readRegister(uint8_t reg) // originally TMP117_Register reg
{
	_i2cPort->beginTransmission(_deviceAddress); // Originally cast (uint8_t)
	_i2cPort->write(reg);
	_i2cPort->endTransmission();					   // endTransmission but keep the connection active
	_i2cPort->requestFrom(_deviceAddress, (uint8_t)2); // Ask for 2 bytes, once done, bus is released by default

	uint8_t data[2] = {0};			// Declares an array of length 2 to be empty
	int16_t datac = 0;				// Declares the return variable to be 0
	if (_i2cPort->available() <= 2) // Won't read more than 2 bits
	{
		data[0] = _i2cPort->read();			// Reads the first set of bits (D15-D8)
		data[1] = _i2cPort->read();			// Reads the second set of bits (D7-D0)
		datac = ((data[0] << 8) | data[1]); // Swap the LSB and the MSB
	}
	return datac;
}

/* WRITE REGISTER
    Wire data to a TMP117 register
*/
void TMP117::writeRegister(uint8_t reg, uint16_t data) // originally TMP117_Register reg
{
	_i2cPort->beginTransmission(_deviceAddress); // Originally cast uint8_t when a register value again
	_i2cPort->write(reg);
	_i2cPort->write(highByte(data)); // Write MSB (D15-D8)
	_i2cPort->write(lowByte(data));  // Write LSB (D7-D0)
	_i2cPort->endTransmission();	 // Stop transmitting data
}

/* READ TEMPERATURE CELSIUS
	This function reads the temperature reading from the sensor
	and returns the value in degrees celsius.
	NOTE: The data type of digitalTemp is a signed integer, meaning that the 
	value of the binary number being read will be negative if the MSB is 1,
	and positive if the bit is 0. 
*/
double TMP117::readTempC()
{
	int16_t digitalTempC = readRegister(TMP117_TEMP_RESULT); // Calls to read registers to pull all the bits to store in an array

	float finalTempC = digitalTempC * TMP117_RESOLUTION; // Multiplies by the resolution for digital to final temp

	return finalTempC;
}

/* READ TEMPERATURE FAHRENHEIT
	This function calculates the fahrenheit reading from the
	celsius reading initially found.
	The device reads in celsius unless this function is called.
*/
double TMP117::readTempF()
{
	return readTempC() * 9.0 / 5.0 + 32.0; // Conversion from °C to °F
}

/* GET TEMPERATURE OFFSET
	This function reads the temperature offset. This reads from the register
	value 0x07 (TMP117_TEMP_OFFSET). This can be found on page 23 of the 
	datasheet. 
*/

bool TMP117::dataReady()
{
	uint16_t response = readRegister(TMP117_CONFIGURATION);

	// If statement to see if the 13th bit of the register is 1 or not
	if (response & 1 << 13)
	{
		return true;
	}
	else
	{
		return false;
	}
}