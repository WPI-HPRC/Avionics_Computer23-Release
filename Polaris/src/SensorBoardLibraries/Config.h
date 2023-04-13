#pragma once
//IMU setup Parameters
//Magnetometer setup Parameters
//Barometer setup Parameters
//MCP2517 setup Parameters

//------------------------ IMU ------------------------//
const uint8_t IMU_I2C_ADDRESS = 0x68;

// IMU Register sets - LOOK AT THE DATASHEET FOR MORE INFO
const uint8_t SENSOR_ENABLE = 0x0F; // 0000 1111 = Enable all sensors

const uint8_t ACCEL_ODR = 0x05; // 0000 0101 = 2 kHz 
const uint8_t ACCEL_FS_SEL = 0x00; // 0000 0000 = +/- 16g

const uint8_t GYRO_ODR = 0x05; // 0000 0101 = 2 kHz
const uint8_t GYRO_FS_SEL = 0x00; // 0000 0000 = +/- 2000 deg/s

// ACCEL AAF: 3dB cutoff frequency: 126 Hz
const uint8_t ACCEL_AAF_DELT = 3;
const uint8_t ACCEL_AAF_DELTSQR = 9; // This limits the max DELTSQR value to 122 but that's fine for our purposes
const uint8_t ACCEL_AAF_BITSHIFT = 12; 

// GYRO AAF 3dB cutoff frequency: 126 Hz
const uint8_t GYRO_AAF_DELT = 3;
const uint8_t GYRO_AAF_DELTSQR = 9; // This limits the max DELTSQR value to 122 but that's fine for our purposes
const uint8_t GYRO_AAF_BITSHIFT = 12;


//------------------------ Magnetometer ------------------------//
const uint8_t MAG_I2C_ADDRESS = 0x30;
const uint8_t Register0_Set = 0x20; // 0010 0000 = 0x20 : Enable Auto set/reset
const uint8_t Register1_Set = 0x03; // 0000 0011 = 0x03 : BW1 = 1, BW0 = 1 -> 800 Hz
const uint8_t Register2_Set = 0xDF; // 1101 1111 = 0xDF : Set CM_Freq to 111 -> 1000Hz, Enable CMM_en, Set Prd_set to 101 -> 500 Measurements/set operation, Enable Auto set/reset

//------------------------ Barometer --------------------------//
const uint8_t BARO_I2C_ADDRESS = 0x77;

// Oversampling ratios
const uint8_t D1_OSR = 0x40; // 0100 0000 = 256 oversampling Pressure
const uint8_t D2_OSR = 0x50; // 0101 0000 = 256 oversampling Temperature

