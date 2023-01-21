#include <EmbeddedSystems\Magnetometer.h>
#include <EmbeddedSystems\MMC5983MA_QFN16_Registers.h>

/*
Read Length bytes starting from SubAddress and store them in data
 Make sure to allocate enough memory for data 
*/
void Magnetometer::ReadRegister(uint8_t SubAddress, uint8_t length, uint8_t *data)//TODO: Check Register bank / switch to right register
{
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->write(SubAddress);
    I2C_BUS->endTransmission(false);
    I2C_BUS->requestFrom(I2C_Address, length);
    for (uint8_t i = 0; i < length; i++)
    {
        data[i] = I2C_BUS->read();
    }
}

/*
Write data to the register at SubAddress and wait 1ms after writing
*/
void Magnetometer::WriteToRegister(uint8_t SubAddress, uint8_t data) //TODO: Check register bank / switch to right register
{
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->write(SubAddress);
    I2C_BUS->write(data);
    I2C_BUS->endTransmission(true);
    delay(1);
}

/*
Write data to the register at SubAddress and wait TimeToDelayAfterWrite ms after writing
*/
void Magnetometer::WriteToRegister(uint8_t SubAddress, uint8_t data, uint32_t TimeToDelayAfterWrite){
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->write(SubAddress);
    I2C_BUS->write(data);
    I2C_BUS->endTransmission(true);
    delay(TimeToDelayAfterWrite);
}

/*
Main setup function for the Magnetometer. This function will set the Magnetometer to the desired settings.
    1. Set Hard and Soft Iron calibration values
    2. Check the who am I register to make sure the Magnetometer is connected and working
    3. Enable the sensor via enable function
*/
void Magnetometer::setup(){
    Magnetometer_HardIron_Values.x = (int)1.3026e+5;
    Magnetometer_HardIron_Values.y = 128618;
    Magnetometer_HardIron_Values.z = 134346;
    Magnetometer_SoftIron_Values.x = .9473f * .0000625f; // .9473f is the scale factor for the magnetometer and .0000625 is the conversion factor from bits to Gauss
    Magnetometer_SoftIron_Values.y = 1.0475f * .0000625f;
    Magnetometer_SoftIron_Values.z = 1.0104f * .0000625f;
    if(CheckWhoAmI()){
        Serial.println("MMC5983MA_QFN16 found");
    }
    else{
        Serial.println("MMC5983MA_QFN16 not found");
    }

    if(SensorEnable()){
        Serial.println("Sensor Enabled in Continuous Mode");
    }
    else{
        Serial.println("Sensor not enabled");
    }

}

/*
    Reset the Magnetometer via writing the reset command to internal control register 1
*/
void Magnetometer::reset(){
    // I2C_BUS->beginTransmission(I2C_Address);
    // I2C_BUS->write(Internal_Control_1);
    // I2C_BUS->write(Reset);
    // I2C_BUS->endTransmission();
    // delay(15);
    WriteToRegister(Internal_Control_1, Reset,15);
}

/*
    Enables the sensor setting the internal control registers
*/
bool Magnetometer::SensorEnable(){
    reset(); //reset the sensor
    
    this->WriteToRegister(Internal_Control_0,Register0Set);
    // I2C_BUS->beginTransmission(I2C_Address);
    // I2C_BUS->write(Internal_Control_0);
    // I2C_BUS->write(Register0Set);
    // I2C_BUS->endTransmission();
    // delay(1);

    this->WriteToRegister(Internal_Control_1,Register1Set);
    // I2C_BUS->beginTransmission(I2C_Address);
    // I2C_BUS->write(Internal_Control_1);
    // I2C_BUS->write(Register1Set);
    // I2C_BUS->endTransmission();
    // delay(1);

    this->WriteToRegister(Internal_Control_2,Register2Set);
    // I2C_BUS->beginTransmission(I2C_Address);
    // I2C_BUS->write(Internal_Control_2);
    // I2C_BUS->write(Register2Set);
    // I2C_BUS->endTransmission();
    // delay(1);
    
    return true;
}

/*
    Reads the magnetometer data from the sensor by:
    1. Stores the raw data into the Raw_Values vector
        a. This sensor has 18 bit resolution so we need to read 7 bytes for three axis since there is the MSB and second MSB then the last two bits
        for each axis: [MSB, Second MSB, two extra bits]
        b. the extra two bits for all three axes are stored in a seperate register at the address directly after the second MSB of the z axis
        c. The extra two bits are stored in the following order: [x axis, y axis, z axis, reserved]
        see the datasheet for more information
    2. Applies the hard and soft iron calibration to the data and stores in the Calibrated_Values vector
*/
void Magnetometer::readSensor(){
    // I2C_BUS->beginTransmission(I2C_Address);
    // I2C_BUS->write(0x00);
    // I2C_BUS->endTransmission(false);
    // I2C_BUS->requestFrom(I2C_Address,7,true);
    // for (int i = 0; i < 7; i++) {
    //     buffer[i] = I2C_BUS->read();
    // }
    this->ReadRegister(0x00,7,buffer);
    Magnetometer_Raw_Values.x = (int32_t) (buffer[0] << 10) | (buffer[1] << 2) | ((buffer[6] & (Bit7 | Bit6))>>6);
    Magnetometer_Raw_Values.y = (int32_t) (buffer[2] << 10) | (buffer[3] << 2) | ((buffer[6] & (Bit5 | Bit4))>>4);
    Magnetometer_Raw_Values.z = (int32_t) (buffer[4] << 10) | (buffer[5] << 2) | ((buffer[6] & (Bit3 | Bit2))>>2);
    Magnetometer_Calibrated_Values.x = (Magnetometer_Raw_Values.x - Magnetometer_HardIron_Values.x)*Magnetometer_SoftIron_Values.x;
    Magnetometer_Calibrated_Values.y = (Magnetometer_Raw_Values.y - Magnetometer_HardIron_Values.y)*Magnetometer_SoftIron_Values.y;
    Magnetometer_Calibrated_Values.z = (Magnetometer_Raw_Values.z - Magnetometer_HardIron_Values.z)*Magnetometer_SoftIron_Values.z;
}

/*
Checks the who am I register to make sure that we are connected to the right sensor
*/
bool Magnetometer::CheckWhoAmI(){
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->write(0x2F);
    I2C_BUS->endTransmission(false);
    I2C_BUS->requestFrom(I2C_Address, 1);
    uint8_t data = I2C_BUS->read();
    if(data == 48){
        return true;
    }
    else{
        return false;
    }
}