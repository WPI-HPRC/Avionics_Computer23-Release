#pragma once

#include "Arduino.h"
#include <lib/SX12XX/SX127XLT.h>

enum TransceiverState {
    RX, TX
};

class Telemetry {
    public:
        Telemetry();

        void init();
        void onLoop(uint32_t timestamp);
        void setState(TransceiverState state);
    private:
        const uint8_t LORA_CS = 6;
        const uint8_t LORA_RST = 5;

        const uint32_t frequency = 920000000; // 920MHz
        const uint32_t offset = 0; // No offset
        const uint8_t bandwith = LORA_BW_125; // 125Khz
        const uint8_t SpreadingFactor = LORA_SF7; //
        const uint8_t CodeRate = LORA_CR_4_5;
        const uint8_t Optimisation = LDRO_AUTO;

        const int8_t TXPower = 10; // TX power in dBm
        const uint16_t packetDelay = 100;

        TransceiverState state = RX; // Default to RX

        SX127XLT radio;
};