#include "Magnetometer_SB.h"
#include "MMC5983MA_Registers.h"
#include <SensorBoardLibraries/Config.h>

/*
    @brief Constructor for the MMC5983MA class
    @param bus The I2C bus to use
    @param address The I2C address of the Magnetometer
*/
MMC5983MA::MMC5983MA(TwoWire &bus, uint8_t address)
{
    I2C_BUS = &bus;
    I2C_Address = address;
}

/*
    @brief Writes data to a register
    @param SubAddress The register to write to
    @param data The data to write
*/
void MMC5983MA::writeRegister(uint8_t SubAddress, uint8_t data)
{
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->write(SubAddress);
    I2C_BUS->write(data);
    I2C_BUS->endTransmission();
}

/*
    @brief Reads data from a register
    @param SubAddress The register to read from
    @param length The number of bytes to read
    @param data The array to store the data in
*/
void MMC5983MA::readRegister(uint8_t SubAddress, int length, uint8_t *data)
{
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->write(SubAddress);
    I2C_BUS->endTransmission();
    I2C_BUS->requestFrom(I2C_Address, length);
    for (int i = 0; i < length; i++)
    {
        data[i] = I2C_BUS->read();
    }
}

/*
    @brief Checks the Product ID of the Magnetometer
    @return True if the Product ID is correct, false otherwise
*/
bool MMC5983MA::CheckProductID()
{
    uint8_t data;
    readRegister(Product_ID_1, 1, &data);
    if (data == MMC5983MA_Product_ID)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*
    @brief Enables the Magnetometer by writing to the control registers according to the Config.h file
*/
void MMC5983MA::sensorEnable()
{
    writeRegister(Internal_control_0,Register0_Set);
    writeRegister(Internal_control_1,Register1_Set);
    writeRegister(Internal_control_2,Register2_Set);
}

/*
    @brief Sets up the Magnetometer by checking the Product ID and enabling the sensor
    @return True if the setup was successful, false otherwise
*/
bool MMC5983MA::setup()
{
    Serial.println("Initializing Magnetometer");
    if(!CheckProductID())
    {
        return false;
    }
    Serial.println("Magnetometer ID Check Passed");
    Serial.println("Enabling Magnetometer");
    sensorEnable();
    Serial.println("Magnetometer Enabled");
    return true;
}

/*
    @brief Reads the Magnetometer data and stores it in the Data array
    @param Data The array to store the data in
    @param StartIndex The index to start storing the data at
*/
void MMC5983MA::readSensor(uint8_t *Data, int StartIndex)
{
    Data += StartIndex; // Move pointer to start index
    readRegister(Xout0, 7, Data); // Read 6 bytes starting from Magnetic_X_MSB
}