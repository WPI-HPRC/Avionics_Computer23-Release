#pragma once

#include "Arduino.h"
#include <SPIFlash.h>
#include <vector>


class FlashChip {
public:
    FlashChip();
    void init();
    uint32_t nextAddress;
    std::vector<uint32_t> addresses;

    bool initialWrite();
    bool writeStruct(const void *message, const uint8_t dataSize);
    // bool readStruct(void *message, const uint8_t dataSize);
    bool readData();
    

    bool eraseMemory();
private:
    SPIFlash * flash;

};