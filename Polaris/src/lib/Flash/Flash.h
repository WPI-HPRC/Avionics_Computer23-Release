#pragma once

#include "Arduino.h"
#include <SPIFlash.h>


class FlashChip {
public:
    FlashChip();
    void init();
    uint32_t nextAddress;

    String addressString;

    bool writeStruct(String structStringe);
    int rememberAddress();
    // bool readData();
    

    bool eraseMemory();
private:
    SPIFlash * flash;
    uint32_t capacity;
    uint32_t maxPage;

};