/**
 * @file FlashChip.h
 * @author Nikhil Gangaram, Daniel Pearson
 * @brief 
 * @version 0.1
 * @date 2023-04-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "Arduino.h"
#include <SPIFlash.h>

class FlashChip {
public:
    FlashChip();
    void init();

    bool writeStruct(const void *message, const uint8_t dataSize);
    bool eraseMemory();
private:

    SPIFlash * flash;

};
