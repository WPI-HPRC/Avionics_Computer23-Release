#pragma once
#include <Arduino.h>
#include <lib/SX12XX/SX127XLT.h>
// Cube Computer Config.h

//------------------------ Barometer --------------------------//
const uint8_t BARO_I2C_ADDRESS = 0x77;

// Oversampling ratios
const uint8_t D1_OSR = 0x40; // 0100 0000 = 256 oversampling Pressure
const uint8_t D2_OSR = 0x50; // 0101 0000 = 256 oversampling Temperature


//------------------------ Temperature Sensor ------------------------//
const uint8_t TEMP_I2C_ADDRESS = 0x48;

const uint8_t CYCLE_CONVERSION = 0x00;
const uint8_t COVERSION_AVERAGING_MODE = 0U; // 0 = NO AVERAGING, 1 = 8 Averaged Conversions, 2 = 32 Averaged Conversions, 3 = 64 Averaged Conversions

//------------------------ Transmitter  ------------------------//
const uint8_t LORA_CS = 6;
const uint8_t LORA_RST = 5;

const uint32_t frequency = 920000000;
const uint32_t offset = 0;
const uint8_t bandwith = LORA_BW_125;
const uint8_t SpreadingFactor = LORA_SF7;
const uint8_t CodeRate = LORA_CR_4_5;
const uint8_t Optimisation = LDRO_AUTO;

const int8_t TXPower = 10; // TX power in dBm
const uint16_t packetDelay = 100;

