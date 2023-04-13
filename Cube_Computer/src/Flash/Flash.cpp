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

    if(flash.eraseChip()) {
        Serial.println("Chip Erased");
    }else{
        Serial.println("Chip Erase Failed");
    }

}

bool FlashChip::writeStruct(String structString) {

    String data = "";
    nextAddress = flash.getAddress(flash.sizeofStr(structString));
    flash.writeStr(nextAddress, structString, true);
    Serial.print("Temperature Written to Flash: "); Serial.println(structString);

    Serial.println("Populating vector with addresses");
    addresses.push_back(nextAddress);

    flash.readStr(nextAddress, data, true);
    Serial.println("Temperature Read from Flash:"); Serial.println(data);

    return true;
}

bool FlashChip::readData() {

    Serial.println(addresses.size());

    for(int i = 0; i < addresses.size(); i++) {
        String data = "";
        flash.readStr(addresses[i], data, true);
        Serial.print(i); Serial.print("   "); Serial.println(addresses[i]); 
        Serial.print("Data: "); Serial.println(data);
    }
    return true;
}

bool FlashChip::eraseMemory() {
    if(!flash.eraseChip()) {
        return false;
    }
    return true;

}