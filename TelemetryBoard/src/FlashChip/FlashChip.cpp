#include "FlashChip.h"

FlashChip::FlashChip() {
    flash = new SPIFlash(9);
}

void FlashChip::init() {
    Serial.println("[Flash Chip]: Beginning initialization");

    flash->begin();
    
    
    Serial.println("====Flash Chip Configuration====");
    Serial.print("Flash Capacity: "); Serial.println(flash->getCapacity());
    Serial.print("Max Page: "); Serial.println(flash->getMaxPage());
    Serial.println("[Flash Chip]: Initialization Complete");

    flash->eraseChip();
}

bool FlashChip::writeStruct(const void *message, const uint8_t dataSize) {
    uint8_t nextAddress = flash->getAddress(dataSize);
    Serial.print("Address: "); Serial.println(flash->getAddress(dataSize));

    flash->writeAnything(nextAddress, message);

    return true;
}

bool FlashChip::eraseMemory() {
    if(!flash->eraseChip()) {
        return false;
    }
    return true;

}