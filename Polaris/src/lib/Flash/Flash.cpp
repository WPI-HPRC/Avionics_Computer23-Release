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


bool FlashChip::writeStruct(String structString) {
    
    // nextAddress += 256;
    Serial.print("SIZE OF THE PACKET's STRING"); Serial.println(flash->sizeofStr(structString));
    // flash->writeStr(nextAddress, structString);

    return true;
}


bool FlashChip::eraseMemory() {
    if(!flash->eraseChip()) {
        return false;
    }
    return true;

}