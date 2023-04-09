#include "Flash.h"

FlashChip::FlashChip() {
    flash = new SPIFlash(10);
}

void FlashChip::init() {
    Serial.println("[Flash Chip]: Beginning initialization");

    flash->begin();
    
    
    Serial.println("====Flash Chip Configuration====");
    Serial.print("Flash Capacity: "); Serial.println(flash->getCapacity());
    Serial.print("Max Page: "); Serial.println(flash->getMaxPage());
    Serial.println("[Flash Chip]: Initialization Complete");

    if(flash->eraseChip()) {
        Serial.println("Chip Erased");
    }else{
        Serial.println("Chip Erase Failed");
    }
}

bool FlashChip::initialWrite() {
    uint8_t data = 0;
    flash->writeAnything(0, data);
    return true;
}

bool FlashChip::writeStruct(const void *message, const uint8_t dataSize) {
    
    // find and write to an available address
    nextAddress = flash->getAddress(dataSize);
    Serial.print("Address: "); Serial.println(flash->getAddress(dataSize));
    flash->writeAnything(nextAddress, message);

    // store the addresses in a vector 
    addresses.push_back(nextAddress);
        

    return true;
}

bool FlashChip::readData() {
    // iterate through the vector of addresses and read the data
    for(int i = 0; i < addresses.size(); i++) {
        uint8_t data;
        flash->readAnything(addresses[i], data);
        Serial.print("Data: "); Serial.println(data);
    }
    return true;
}

bool FlashChip::eraseMemory() {
    if(!flash->eraseChip()) {
        return false;
    }
    return true;

}