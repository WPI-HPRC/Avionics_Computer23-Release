#pragma once

#include <Arduino.h>
#include <SPIFlash.h>
#include <vector>


class FlashChip {
public:
    FlashChip();
    void init();
    uint32_t nextAddress;
    std::vector<uint32_t> addresses;

    bool writeStruct(String structString);
    bool readData();
    

    bool eraseMemory();
private:
    SPIFlash flash = SPIFlash(7);

};