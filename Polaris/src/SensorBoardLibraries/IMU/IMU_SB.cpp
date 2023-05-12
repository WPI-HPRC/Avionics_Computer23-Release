/*
    @author Samay Govani
    @brief ICM-4288P IMU Class for the Sensor Board
    Worcester Polytechnic Institute High Power Rocketry Club
*/
#include <Arduino.h>
#include <Wire.h>
#include "IMU_SB.h"
#include "ICM42688-P_Registers.h"
#include <SensorBoardLibraries/Config.h>

/*
    @brief Constructor for the ICM42688P class
    @param bus The I2C bus to use
    @param address The I2C address of the IMU
*/
ICM42688P::ICM42688P(TwoWire &bus, uint8_t address){
    I2C_BUS = &bus;
    I2C_Address = address;
}

/*
    @brief Reads the IMU and stores the data in the Data array
    @param Data The array to store the data in
    @param StartIndex The index to start storing the data at
*/
void ICM42688P::readRegister(uint8_t reg, uint8_t length, uint8_t* data){
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->write(reg);
    I2C_BUS->endTransmission(false);
    I2C_BUS->requestFrom(I2C_Address,length);
    for(int i = 0; i < length; i++){
        data[i] = I2C_BUS->read();
    }
}

/*
    @brief Reads the IMU and stores the data in the Data array
    @param data the array to store the data in
    @param length the length of the data array
    @param bank the bank to read from
*/
void ICM42688P::readRegister(uint8_t reg, uint8_t length, uint8_t* data, uint8_t bank){
    // Switch to bank
    if(!switchToBank(bank)){
        // Serial.printf("Failed to switch to bank %d, when reading from register %d, aborting read",bank,reg);
        // Serial.println("Failed to switch to bank ");
        // Serial.print(bank);
        // Serial.print(" when reading from register ");
        // Serial.println(reg);
        return;
    }

    // Read from register
    readRegister(reg,length,data);
}

/*
    @brief Writes data to a register
    @param reg The register to write to
    @param data The data to write
    @return True if the write was successful, false otherwise
    @attention This function does not check if the write was successful
*/
void ICM42688P::writeRegister(uint8_t reg, uint8_t data){
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->write(reg);
    I2C_BUS->write(data);
    I2C_BUS->endTransmission();
    delay(1); // Delay to ensure write is complete
}

/*
    @brief Writes data to a register
    @param reg The register to write to
    @param data The data to write
    @param bank The bank to write to
    @return True if the write was successful, false otherwise
*/
bool ICM42688P::writeRegister(uint8_t reg, uint8_t data, uint8_t bank){
    // Switch to bank
    if(!switchToBank(bank)){
        // Serial.printf("Failed to switch to bank %d, when writing %d to register %d, aborting write",bank,data,reg);
        // Serial.println("Failed to switch to bank ");
        // Serial.print(bank);
        // Serial.print(" when writing ");
        // Serial.print(data);
        // Serial.print(" to register ");
        // Serial.println(reg);
        return false;
    }

    // Write to register
    writeRegister(reg,data);

    // Check if write was successful by reading back
    uint8_t readData;
    readRegister(reg,1,&readData);
    if(readData == data){
        return true;
    }
    else{
        return false;
    }
}

/*
    @brief Switches to a bank
    @param bank The bank to switch to
    @return True if the switch was successful, false otherwise
*/
bool ICM42688P::switchToBank(uint8_t bank){
    // Switch to bank
    writeRegister(REG_BANK_SEL, bank);

    // Check if switch was successful by reading back
    uint8_t readData;
    readRegister(REG_BANK_SEL,1,&readData);
    if(readData == bank){
        return true;
    }
    else{
        return false;
    }
}

/*
    @brief Checks the WHO_AM_I register to see if the IMU is connected
    @return True if the IMU is connected, false otherwise
*/
bool ICM42688P::checkWhoAmI(){
    uint8_t whoAmI;
    readRegister(WHO_AM_I,1,&whoAmI);
    if(whoAmI == 0x47){
        return true;
    }
    else{
        return false;
    }
}

/*
    @brief Turns on the IMU's sensors
    @return True if the IMU was initialized successfully, false otherwise
*/
bool ICM42688P::sensorEnable(){
    bool retval =  writeRegister(PWR_MGMT0,SENSOR_ENABLE,0);
    delayMicroseconds(200);
    return retval;
}

/*
    @brief Sets the ODR and FS of the IMU according the Config.h file
    @return True if the ODR and FS were set successfully, false otherwise
*/
bool ICM42688P::setODR_FS(){
    if(!writeRegister(ACCEL_CONFIG0,ACCEL_ODR | ACCEL_FS_SEL,0)){
        return false;
    }
    if(!writeRegister(GYRO_CONFIG0,GYRO_ODR | GYRO_FS_SEL,0)){
        return false;
    }
    return true;
}

/*
    @brief Sets the filters of the IMU according to the Config.h file
    @return True if the filters were set successfully, false otherwise
*/
bool ICM42688P::setFilters(){
    uint8_t RegisterVal = 0;

    // Accelerometer
    readRegister(ACCEL_CONFIG_STATIC2,1,&RegisterVal,2);
    // Clear the 6:1 bits of the register 
    RegisterVal &= ~(0x3F << 1);
    // Set the 6:1 bits of the register to the desired value
    RegisterVal |= (ACCEL_AAF_DELT << 1);
    // Set the 0th bit of the register to 1 to enable the filter
    RegisterVal |= 0x01;
    if(!writeRegister(ACCEL_CONFIG_STATIC2,RegisterVal,2)){
        return false;
    }

    if(!writeRegister(ACCEL_CONFIG_STATIC3,ACCEL_AAF_DELTSQR,2)){
        return false;
    }

    if(!writeRegister(ACCEL_CONFIG_STATIC4,(ACCEL_AAF_BITSHIFT << 4),2)){ // Set the 7:4 bits of the register to the configured value, leave the 3:0 bits as 0 since the DELTSQR value is limited to only an 8 bit integer
        return false;
    }

    // Gyroscope
    readRegister(GYRO_CONFIG_STATIC2,1,&RegisterVal,1);
    if (!writeRegister(GYRO_CONFIG_STATIC2,RegisterVal | 0x03,1)){ // Set the 1:0 bits of the register to 0b11 to enable the filter
        return false;
    }

    readRegister(GYRO_CONFIG_STATIC3,1,&RegisterVal,1);
    // Clear the 5:0 bits of the register
    RegisterVal &= ~(0x3F);
    // Set the 5:0 bits of the register to the desired value
    RegisterVal |= GYRO_AAF_DELT;
    if(!writeRegister(GYRO_CONFIG_STATIC3,RegisterVal,1)){
        return false;
    }

    if(!writeRegister(GYRO_CONFIG_STATIC4,GYRO_AAF_DELTSQR,1)){ 
        return false;
    }

    if(!writeRegister(GYRO_CONFIG_STATIC5,(GYRO_AAF_BITSHIFT << 4),1)){ // Set the 7:4 bits of the register to the configured value, leave the 3:0 bits as 0 since the DELTSQR value is limited to only an 8 bit integer
        return false;
    }

    return true;
}

/*
    @brief Sets up the IMU by checking the WHO_AM_I register, enabling the sensors, setting the ODR and FS, and setting the filters
    @return True if the IMU was setup successfully, false otherwise
*/
bool ICM42688P::setup(){
    if(!checkWhoAmI()){
        Serial.println("IMU:Who am I check failed");
        return false;
    }
    Serial.println("IMU:Who am I check passed");
    if(!sensorEnable()){
        Serial.println("IMU:Sensor enable failed");
        return false;
    }
    Serial.println("IMU:Sensor enable passed");
    if(!setODR_FS()){
        Serial.println("IMU:Set ODR and FS failed");
        return false;
    }
    Serial.println("IMU:Set ODR and FS passed");
    if(!setFilters()){
        Serial.println("IMU:Set filters failed");
        return false;
    }
    Serial.println("IMU:Set filters passed");

    Serial.println("IMU:Switching to bank 0");
    if (!switchToBank(0)){
        Serial.println("IMU:Failed to switch to bank 0");
        return false;
    }
    //TODO:: Read data from the sensor to make sure it is working

    Serial.println("IMU Setup complete");

    return true;
}

/*
    @brief Reads the data from the IMU (12 Bytes) and stores it in the Data array
    @param Data The array to store the data in
    @param StartIndex The index to start storing the data at
    @details The data is stored in the following order:
        - Accel_Data_X [15:8]
        - Accel_Data_X [7:0]
        - Accel_Data_Y [15:8]
        - Accel_Data_Y [7:0]
        - Accel_Data_Z [15:8]
        - Accel_Data_Z [7:0]
        - Gyro_Data_X [15:8]
        - Gyro_Data_X [7:0]
        - Gyro_Data_Y [15:8]
        - Gyro_Data_Y [7:0]
        - Gyro_Data_Z [15:8]
        - Gyro_Data_Z [7:0]
*/  
void ICM42688P::readSensor(uint8_t *Data, int StartIndex){
    // shift the pointer of the array to the start index
    Data = Data + StartIndex;
    // read the data
    this -> readRegister(ACCEL_DATA_X1,12,Data);
}

/*
    @brief Processes the high and low bytes of the IMU data into a 16 bit integer
    @param highByte The high byte of the data
    @param lowByte The low byte of the data
    @return The 16 bit integer of the data
*/
int16_t ICM42688P::processHighLowByte(uint8_t highByte, uint8_t lowByte){
    int16_t retval = (highByte << 8) | lowByte;
    return retval;
}