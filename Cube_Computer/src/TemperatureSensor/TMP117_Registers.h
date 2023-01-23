#include <Arduino.h>

const uint8_t TEMP_RESULT = 0x00; // Read only
const uint8_t TEMP_CONFIG = 0x01; // Read/Write
const uint8_t TEMP_HIGH_LIMIT = 0x02; // Read/Write
const uint8_t TEMP_LOW_LIMIT = 0x03; // Read/Write
const uint8_t EEPROM_UL = 0x04; // Read/Write
const uint8_t EEPROM1 = 0x05; // Read/Write
const uint8_t EEPROM2 = 0x06; // Read/Write
const uint8_t TEMP_OFFSET = 0x07; // Read/Write
const uint8_t EEPROM3 = 0x08; // Read/Write 
const uint8_t DEVICE_ID = 0x0F; // Read only