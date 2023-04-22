#include "Flash.h"

FlashChip::FlashChip() {
    flash = new SPIFlash(10);
}

void FlashChip::init() {
    Serial.println("[Flash Chip]: Beginning initialization");

    flash->begin();

    capacity = flash->getCapacity();
    maxPage = flash->getMaxPage();
    
    
    Serial.println("====Flash Chip Configuration====");
    Serial.print("Flash Capacity: "); Serial.println(capacity);
    Serial.print("Max Page: "); Serial.println(maxPage);
    Serial.println("[Flash Chip]: Initialization Complete");


}

int FlashChip::rememberAddress() {
    // while(1) {
    //     uint32_t tempAddress = (b1 << 24) | (b2 << 16) | (b3 << 8) | b4;

    //     if(tempAddress != -1) {
    //         nextAddress = tempAddress;
    //         break;
    //     }
    // }
    // while(flash->readULong(0) == -1) {
    //     nextAddress = flash->readULong(0);
    // }

    // nextAddress = flash->readULong(0);

    for(int i=0; i < maxPage; i++) {
        uint32_t pageAddress = 0 + (i * 256);

        uint8_t * buffer = new uint8_t[255];

        for(int j=0; j < 255; j++) {
            buffer[j] = flash->readByte(pageAddress + j);
        }
        
        // for(int j = 0; j < 255; j++) {
        //     Serial.print(buffer[j]);

        //     if(j < 254) {
        //         Serial.print(", ");
        //     } else {
        //         Serial.println();
        //     }

        // }

        bool isUnwritten = true;
        for(uint32_t k = 0; k < 255; k++) {
            if(buffer[k] != 255) {
                isUnwritten = false;
                break;
            }
        }
        
        delete[] buffer;

        if(isUnwritten) {
            nextAddress = pageAddress;
            return pageAddress;
        }
    }

    return 0;
}

bool FlashChip::writeStruct(String structString) {
    if(nextAddress == capacity) {
        return false;
    }
    
    flash->writeStr(nextAddress, structString);
    // flash->eraseSection(0,4);

    nextAddress += 256;

    // flash->writeULong(0, nextAddress);

    // uint32_t readAddress = flash->readULong(0);
    // Serial.print("Read Address: "); Serial.println(readAddress);

    return true;
}


bool FlashChip::eraseMemory() {
    if(!flash->eraseChip()) {
        return false;
    }
    return true;

}