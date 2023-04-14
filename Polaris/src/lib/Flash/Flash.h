#pragma once

#include "Arduino.h"
#include <SPIFlash.h>


class FlashChip {
public:
    FlashChip();
    void init();
    uint32_t nextAddress;
    bool writeStruct(String structStringe);
    // bool readData();
    

    bool eraseMemory();
private:
    SPIFlash * flash;

};