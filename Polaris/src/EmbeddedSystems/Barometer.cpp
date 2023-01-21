#include "EmbeddedSystems\Barometer.h"

/*
    * @brief constructor for the barometer class
    * @param bus I2C bus that the barometer is connected to
    * @param address I2C address of the barometer ms5611 
*/
Barometer::Barometer(TwoWire &bus,uint8_t address)
{
    this->I2C_Address = address;
    this->I2C_BUS = &bus;
}

/*
    * @brief setup the barometer
    * @return true if the barometer was setup correctly
*/
bool Barometer::setup(){
    this->reset();
    this->ReadCalibrationData();
    //TODO add calibration routine for AGL and ASL altitudes
    return true;
}

/*
    * @brief Reads the PROM on the barometer and stores the calibration data in the calibrationData struct
*/
void Barometer::ReadCalibrationData(){
    uint8_t Buffer[2] = {};
    
    sendCommand(PROM_Data_Address1);
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,2);
    Buffer[0] = Wire.read();
    Buffer[1] = Wire.read();
    this->calibrationData.SensT1 = (uint16_t) ((Buffer[0] << 8)|Buffer[1]);

    sendCommand(PROM_Data_Address2);
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,2);
    Buffer[0] = Wire.read();
    Buffer[1] = Wire.read();
    this->calibrationData.OFFT1 = (uint16_t) ((Buffer[0] << 8)|Buffer[1]);

    sendCommand(PROM_Data_Address3);
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,2);
    Buffer[0] = Wire.read();
    Buffer[1] = Wire.read();
    this->calibrationData.TCS = (uint16_t) ((Buffer[0] << 8)|Buffer[1]);

    sendCommand(PROM_Data_Address4);
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,2);
    Buffer[0] = Wire.read();
    Buffer[1] = Wire.read();
    this->calibrationData.TCO = (uint16_t) ((Buffer[0] << 8)|Buffer[1]);
    
    sendCommand(PROM_Data_Address5);
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,2);
    Buffer[0] = Wire.read();
    Buffer[1] = Wire.read();
    this->calibrationData.Tref = (uint16_t) ((Buffer[0] << 8)|Buffer[1]);
    
    sendCommand(PROM_Data_Address6);
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,2);
    Buffer[0] = Wire.read();
    Buffer[1] = Wire.read();
    this->calibrationData.TEMPSENS = (uint16_t) ((Buffer[0] << 8)|Buffer[1]);

    Serial.print("C1:");
    Serial.println(this->calibrationData.SensT1);
    Serial.print("C2:");
    Serial.println(this->calibrationData.OFFT1);
    Serial.print("C3:");
    Serial.println(this->calibrationData.TCS);
    Serial.print("C4:");
    Serial.println(this->calibrationData.TCO);
    Serial.print("C5:");
    Serial.println(this->calibrationData.Tref);
    Serial.print("C6:");
    Serial.println(this->calibrationData.TEMPSENS);

}

/*
    * @brief Sends a command to the barometer
    * @param command sent to the barometer
*/
void Barometer::sendCommand(uint8_t command){
	Wire.beginTransmission(this->I2C_Address);
	Wire.write(command);
	Wire.endTransmission();
    return;
}

/*
    * @brief Resets the barometer
*/
void Barometer::reset(){
    sendCommand(RESET);
    delayMicroseconds(2800); //wait for the reset to complete (datasheet says 2800us)
    return;
}

/*
    * @brief Reads the raw temperature data from the barometer
*/
void Barometer::readRawTemperature(){
    sendCommand(D1_OSR_256); //Use the fastest oversampling ratio
    delayMicroseconds(600); //Wait for the conversion to complete (Datasheet says 600us)
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,3);
    Buffer[0] = Wire.read(); //XSB
    Buffer[1] = Wire.read(); //MSB
    Buffer[2] = Wire.read(); //LSB
    this->RawTemperatureData = (uint32_t)((Buffer[0] << 16) | (Buffer[1] << 8) | Buffer[2]);
    return;
}

/*
    * @brief Reads the raw pressure data from the barometer
*/
void Barometer::readRawPressure(){
    sendCommand(D2_OSR_256); //Use the fastest oversampling ratio
    delayMicroseconds(600); //Wait for the conversion to complete (Datasheet says 600us)
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,3);
    Buffer[0] = Wire.read(); //XSB
    Buffer[1] = Wire.read(); //MSB
    Buffer[2] = Wire.read(); //LSB
    this->RawPressureData = (uint32_t)((Buffer[0] << 16) | (Buffer[1] << 8) | Buffer[2]);
    return;
}

/*
    * @brief Collects the raw temperature and pressure data from the barometer
*/
void Barometer::CollectRawData(){
    readRawTemperature();
    readRawPressure();
    return;
}

/*
    * @brief
         1. Calculates the temperature from the raw temperature data
         2. Calculates the temperature compensated pressure from the raw pressure and temperature data
         page 8 of the datasheet
*/
void Barometer::ProcessRawData(){
    int32_t dT = ((int64_t)this->RawTemperatureData - (int64_t)(this->calibrationData.Tref))<<8;
    // Serial.print("dT:");
    // Serial.println(dT);
    int32_t TEMP = (uint32_t)2000 + ((dT * (int32_t)this->calibrationData.TEMPSENS) >> 23);
    // Serial.print("TEMP:");
    // Serial.println(TEMP);
    int64_t OFF = ((int64_t)this->calibrationData.OFFT1 << 16) + (((int64_t)this->calibrationData.TCO * dT) >> 7);
    // Serial.print("OFF:");
    // Serial.println(OFF);
    int64_t SENS = ((int64_t)this->calibrationData.SensT1 << 15) + (((int64_t)this->calibrationData.TCS * dT) >> 8);
    // Serial.print("SENS:");
    // Serial.println(SENS);
    int32_t P = ((((int64_t)this->RawPressureData * SENS) >> 21) - OFF) >> 15;
    // Serial.print("P:");
    // Serial.println(P);
    this->Temperature = (float)(TEMP/100);
    this->CompensatedPressure = (float)(P/100);
    // Serial.print("Calculated Temperature:");
    // Serial.print(this->Temperature);
    // Serial.print(",");
    // Serial.print("Calculated Pressure:");
    // Serial.println(this->CompensatedPressure);
    return;
}

/*
    * @brief Collects and processes the raw data from the barometer
*/
void Barometer::readSensor(){
    CollectRawData();
    ProcessRawData();
}

/**
 * @brief Getter for calibrated pressure
 * 
 * @return float CompensatedPressure
 */
float Barometer::getCalPressure() {
    return CompensatedPressure;
}

/**
 * @brief Getter for calibrated temperature
 * 
 * @return float Temperature
 */
float Barometer::getCalTemperature() {
    return Temperature;
}