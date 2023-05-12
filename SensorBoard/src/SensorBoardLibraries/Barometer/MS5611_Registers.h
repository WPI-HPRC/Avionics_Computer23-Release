#include <Arduino.h>

const uint8_t RESET = 0x1E;
const uint8_t ADC_READ = 0x00;
const int NUM_BYTES_ADC_READ = 3;
const int NUM_BYTES_PROM_DATA = 2;

const uint8_t PROM_READ_ADDRESS_0 = 0xA0;
const uint8_t PROM_READ_ADDRESS_1 = 0xA2;
const uint8_t PROM_READ_ADDRESS_2 = 0xA4;
const uint8_t PROM_READ_ADDRESS_3 = 0xA6;
const uint8_t PROM_READ_ADDRESS_4 = 0xA8;
const uint8_t PROM_READ_ADDRESS_5 = 0xAA;
const uint8_t PROM_READ_ADDRESS_6 = 0xAC;
const uint8_t PROM_READ_ADDRESS_7 = 0xAE;