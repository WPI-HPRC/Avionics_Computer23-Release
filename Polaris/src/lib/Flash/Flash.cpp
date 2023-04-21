#include "Flash.h"

FlashChip::FlashChip() {
}

void FlashChip::init() {
    Serial.println("[Flash Chip]: Beginning initialization");

    flash.begin();
    
    Serial.println("====Flash Chip Configuration====");
    Serial.print("Flash Capacity: "); Serial.println(flash.getCapacity());
    Serial.print("Max Page: "); Serial.println(flash.getMaxPage());
    Serial.println("[Flash Chip]: Initialization Complete");

}

void FlashChip::rememberAddress() {
    flash.readStr(0, read);
    nextAddress = (uint32_t)(read.toInt());
}

bool FlashChip::writeStruct(String structString) {
    
    // Writes the struct to the flash chip
    // and increments the nextAddress variable
    flash.writeStr(nextAddress, structString);
    nextAddress += 256;

    // updates the nextAddressString variable
    // and writes it to the flash chip's first page
    nextAddressString = String(nextAddress);
    flash.eraseSection(0, 256);
    flash.writeStr(0, nextAddressString);

    return true;
}
