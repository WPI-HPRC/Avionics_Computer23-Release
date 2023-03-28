#include "Barometer_SB.h"
#include "Wire.h"
#include "MS5611_Registers.h"
#include <SensorBoardLibraries/Config.h>

/*
    @brief Constructor for the MS5611 class
    @param bus The I2C bus to use
    @param address The I2C address of the Barometer
*/
MS5611::MS5611(TwoWire &bus,uint8_t address){
    I2C_BUS = &bus;
    I2C_Address = address;
}

/*
    @brief Sends a command to the sensor
    @param 8-bit command The command to send
*/
void MS5611::sendCommand(uint8_t command){
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->write(command);
    I2C_BUS->endTransmission();
}

/*
    @brief Sets up the sensor
    @return True if the sensor is setup correctly, false otherwise
*/
bool MS5611::setup(){
    // Serial.println("Initializing Barometer");
    this->readCalibrationData();
    // Serial.println("Resetting Barometer");
    sendCommand(RESET);
    delay(10);
    // Serial.println("Barometer Reset Complete");
    // Serial.println("Barometer Setup Complete");
    //TODO Read data from the sensor to make sure it is working
    return true;
}

/*
    @brief Reads the sensor data and stores it in the Data array at the StartIndex - 6 bytes of data are stored starting at StartIndex
    @param Data - The array to store the sensor data in
    @param StartIndex - The index to start storing the sensor data at
*/
void MS5611::readSensor(uint8_t *Data, int StartIndex){
    sendCommand(D1_OSR);
    delayMicroseconds(600); // 600 us delay for 256 oversampling ratio change as per datasheet, this is what the datasheet says to be the max ADC conversion time for 256 oversampling ratio
    sendCommand(ADC_READ);

    // Read ADC data from the sensor and store it in the Data array
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->requestFrom(I2C_Address,NUM_BYTES_ADC_READ);
    for(int i = 0; i < NUM_BYTES_ADC_READ; i++){
        Data[StartIndex + i] = I2C_BUS->read();
    }
    I2C_BUS->endTransmission();

    sendCommand(D2_OSR);
    delayMicroseconds(600); // 600 us delay for 256 oversampling ratio change as per datasheet
    sendCommand(ADC_READ);

    // Read ADC data from the sensor and store it in the Data array
    I2C_BUS->beginTransmission(I2C_Address);
    I2C_BUS->requestFrom(I2C_Address,NUM_BYTES_ADC_READ);
    for(int i = 0; i < NUM_BYTES_ADC_READ; i++){
        Data[StartIndex + NUM_BYTES_ADC_READ + i] = I2C_BUS->read();
    }

    // Serial.print("D2: ");
    // Serial.println(processHighMidLowByte(Data[StartIndex + NUM_BYTES_ADC_READ],Data[StartIndex + NUM_BYTES_ADC_READ + 1],Data[StartIndex + NUM_BYTES_ADC_READ + 2]));
}

void MS5611::readCalibrationData(){
    uint8_t Buffer[2] = {};
    
    sendCommand(PROM_READ_ADDRESS_1);
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,NUM_BYTES_PROM_DATA);
    Buffer[0] = Wire.read();
    Buffer[1] = Wire.read();
    this->calibrationData.C1 = ((uint16_t)((Buffer[0] << 8)|Buffer[1]));
    // Serial.print("SENS_T1: ");
    // Serial.println((uint16_t)((Buffer[0] << 8)|Buffer[1]));

    sendCommand(PROM_READ_ADDRESS_2);
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,NUM_BYTES_PROM_DATA);
    Buffer[0] = Wire.read();
    Buffer[1] = Wire.read();
    this->calibrationData.C2 = ((uint16_t)((Buffer[0] << 8)|Buffer[1]));
    // Serial.print("OFFT1: ");
    // Serial.println(((uint16_t)((Buffer[0] << 8)|Buffer[1])));

    sendCommand(PROM_READ_ADDRESS_3);
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,NUM_BYTES_PROM_DATA);
    Buffer[0] = Wire.read();
    Buffer[1] = Wire.read();
    this->calibrationData.C3 = ((uint16_t)((Buffer[0] << 8)|Buffer[1]));
    // Serial.print("TCS: ");
    // Serial.println(((uint16_t)((Buffer[0] << 8)|Buffer[1])));

    sendCommand(PROM_READ_ADDRESS_4);
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,NUM_BYTES_PROM_DATA);
    Buffer[0] = Wire.read();
    Buffer[1] = Wire.read();
    this->calibrationData.C4 = ((uint16_t)((Buffer[0] << 8)|Buffer[1]));
    // Serial.print("TCO: ");
    // Serial.println(((uint16_t)((Buffer[0] << 8)|Buffer[1])));
    
    sendCommand(PROM_READ_ADDRESS_5);
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,NUM_BYTES_PROM_DATA);
    Buffer[0] = Wire.read();
    Buffer[1] = Wire.read();
    this->calibrationData.C5 = ((uint16_t)((Buffer[0] << 8)|Buffer[1]));
    // Serial.print("Tref: ");
    // Serial.println(((uint16_t)((Buffer[0] << 8)|Buffer[1])));

    sendCommand(PROM_READ_ADDRESS_6);
    Wire.beginTransmission(this->I2C_Address);
    Wire.requestFrom(this->I2C_Address,NUM_BYTES_PROM_DATA);
    Buffer[0] = Wire.read();
    Buffer[1] = Wire.read();
    this->calibrationData.C6 = ((uint16_t)((Buffer[0] << 8)|Buffer[1]));
    // Serial.print("TEMPSENS: ");
    // Serial.println(((uint16_t)((Buffer[0] << 8)|Buffer[1])));
    // Serial.println("Calibration Data Read Complete");
    Wire.endTransmission();
}