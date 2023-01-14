#include <Arduino.h>

const uint8_t WRITE_ENABLE = 0x06;
const uint8_t VOLATILE_SR_WRITE_ENABLE = 0x50;
const uint8_t WRITE_DISABLE = 0x04;

const uint8_t RELEASE_POWER_DOWN_ID = 0xAB;
const uint8_t READ_MANUFACTURER_ID = 0x90;
const uint8_t READ_JEDEC_ID = 0x9F;
const uint8_t READ_UNIQUE_ID = 0x4B;

const uint8_t READ_DATA = 0x03;
const uint8_t FAST_READ = 0x0B;

const uint8_t PAGE_PROGRAM = 0x02;

const uint8_t SECTOR_ERASE_4KB = 0x20;
const uint8_t BLOCK_ERASE_32KB = 0x52;
const uint8_t BLOCK_ERASE_64KB = 0xD8;
const uint8_t CHIP_ERASE = 0xC7;

const uint8_t READ_STATUS_REGISTER_1 = 0x05;
const uint8_t READ_STATUS_REGISTER_2 = 0x35;
const uint8_t READ_STATUS_REGISTER_3 = 0x15;

const uint8_t WRITE_STATUS_REGISTER_1 = 0x01;
const uint8_t WRITE_STATUS_REGISTER_2 = 0x31;
const uint8_t WRITE_STATUS_REGISTER_3 = 0x11;

const uint8_t READ_SFDP_REGISTER = 0x5A;
const uint8_t ERASE_SECURITY_REGISTER = 0x44;
const uint8_t PROGRAM_SECURITY_REGISTER = 0x42;
const uint8_t READ_SECURITY_REGISTER = 0x48;

const uint8_t GLOBAL_BLOCK_LOCK = 0x7E;
const uint8_t GLOBAL_BLOCK_UNLOCK = 0x98;
const uint8_t READ_BLOCK_LOCK = 0x3D;
const uint8_t INDIVIDUAL_BLOCK_LOCK = 0x36;
const uint8_t INDIVIDUAL_BLOCK_UNLOCK = 0x39;

const uint8_t ERASE_PROGRAM_SUSPEND = 0x75;
const uint8_t ERASE_PROGRAM_RESUME = 0x7A;

const uint8_t POWER_DOWN = 0xB9;

const uint8_t ENABLE_RESET = 0x66;
const uint8_t RESET_DEVICE = 0x99;
