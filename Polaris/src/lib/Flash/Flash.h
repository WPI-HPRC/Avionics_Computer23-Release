#pragma once

#include "Arduino.h"
#include <SPIFlash.h>


class FlashChip {

public:
    FlashChip();
    uint32_t nextAddress;
    String nextAddressString;
    String read;
    SPIFlash flash = SPIFlash(10);

    void init();
    bool writeStruct(String structStringe);
    void rememberAddress();
};