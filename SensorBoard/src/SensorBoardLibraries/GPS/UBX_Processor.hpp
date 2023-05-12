#include <Arduino.h>
class UBX_Processor
{
    static bool isChecksumValid(const uint8_t* data, uint16_t length);
};