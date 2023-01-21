#include <EmbeddedSystems\IMU.h>
#include <EmbeddedSystems\ICM42688_Registers.h>

// Write data to the register at SubAddress 
void IMU::WriteToRegister(uint8_t SubAddress, uint8_t data) //TODO: Check register bank / switch to right register
{
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->write(SubAddress);
    I2C_BUS->write(data);
    I2C_BUS->endTransmission(true);
    delay(1);
}

/*
Read Length bytes starting from SubAddress and store them in data
Make sure to allocate enough memory for data 
@param SubAddress register to start reading from
@param Length number of bytes to read
@param data pointer to the array to store the data in
*/
void IMU::ReadRegister(uint8_t SubAddress, uint8_t length, uint8_t *data) //TODO: Check Register bank / switch to right register
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
Main setup function for the IMU. This function will set the IMU to the desired settings.
    1. Check the who am I register to make sure the IMU is connected and working
    2. Enable the sensor via the PWR management register
    3. Set the accelerometer and gyroscope to the desired settings
*/
void IMU::setup(){
    // Check to see if we have found the right sensor
    if(CheckWhoAmI()){
        Serial.println("ICM 42688-P found");
    }
    else{
        Serial.println("ICM 42688-P not found");
    }
    // Enable the sensor
    if(SensorEnable()){
        Serial.println("Sensor Enabled");
    }
    else{
        Serial.println("Sensor not enabled");
    }
    //TODO: Set the accelerometer and gyroscope to the desired settings:
    // 1. Set the accelerometer range if desired
    // 2. Set the gyroscope range if desired
    // 3. Set the accelerometer and gyroscope to the desired sample rate if desired
    // 4. Set the accelerometer and gyroscope to the desired bandwidth for filtering if desired

    //TODO: These are currently default values for the accelerometer and gyroscope ranges
    this->Accelerometer_Range = ACCEL_16G;
    this -> RateGyro_Range = GYRO_2000DPS;
}

/*
Enables the sensor using the PWR_MGMT_0 register
*/
bool IMU::SensorEnable(){
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->write(PWR_MGMT0);
    I2C_BUS->write(SEN_ENABLE);
    I2C_BUS->endTransmission(true);
    delay(1);
    
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->write(PWR_MGMT0);
    I2C_BUS->endTransmission(false);
    I2C_BUS->requestFrom(I2C_Address, 1);
    uint8_t data = I2C_BUS->read();
    if (data == 15){
        return true;
    }
    else{
        return false;
    }
}

/*
Checks the who am I register to make sure that we are connected to the right sensor
*/
bool IMU::CheckWhoAmI(){
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->write(WHO_AM_I);
    I2C_BUS->endTransmission(false);
    I2C_BUS->requestFrom(I2C_Address, 1);
    uint8_t data = I2C_BUS->read();
    if(data == 71){
        return true;
    }
    else{
        return false;
    }
}

/*
Constructs IMU object with the given I2C bus and address
@param bus I2C bus that the IMU is connected to
@param address I2C address of the ICM 42688-P
*/
IMU::IMU(TwoWire &bus,uint8_t address)
{
    I2C_BUS = &bus;
    I2C_Address = address;
}

/*
Takes in two unsigned 8 bit integers (representing the high and low byte) and returns a signed 16 bit integer
@param highByte high byte of the 16 bit integer
@param lowByte low byte of the 16 bit integer
*/
int16_t processHighLowBytes(uint8_t Hbyte, uint8_t Lbyte){
	return (int16_t) ((Hbyte << 8)|Lbyte); // convert to a 16 bit signed integer from the high and low bytes of the data
}

/*
Reads the accelerometer and gyroscope data from the IMU and stores it in the buffer array in the form:
    Each data value is a signed 16 bit integer stored as two unsigned 8 bit integers
    [Accel_X_H, Accel_X_L, Accel_Y_H, Accel_Y_L, Accel_Z_H, Accel_Z_L, Gyro_X_H, Gyro_X_L, Gyro_Y_H, Gyro_Y_L, Gyro_Z_H, Gyro_Z_L]
    then processes the High and Low bytes and stores into the raw value vectors
*/
void IMU::readSensor(){
    // I2C_BUS->beginTransmission(I2C_Address);
    // I2C_BUS->write(TEMP_OUT);
    // I2C_BUS->endTransmission(false);
    // I2C_BUS->requestFrom(I2C_Address,14);
    // for(uint8_t i = 0; i<14;i++){
    //     Buffer[i] = I2C_BUS->read();
    // }
    this->ReadRegister(ACCEL_OUT,12,Buffer);
    Accelerometer_Raw_Values.x = processHighLowBytes(Buffer[0],Buffer[1]);
    Accelerometer_Raw_Values.y = processHighLowBytes(Buffer[2],Buffer[3]);
    Accelerometer_Raw_Values.z = processHighLowBytes(Buffer[4],Buffer[5]);
    Gyro_Raw_Values.x = processHighLowBytes(Buffer[6],Buffer[7]);
    Gyro_Raw_Values.y = processHighLowBytes(Buffer[8],Buffer[9]);
    Gyro_Raw_Values.z = processHighLowBytes(Buffer[10],Buffer[11]);
}

Vector3<int16_t> IMU::getAccelVals() {
    return Accelerometer_Raw_Values;
}
