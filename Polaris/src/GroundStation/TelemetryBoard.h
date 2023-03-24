/**
 * @file TelemetryBoard.h
 * @author Daniel Pearson
 * @brief Telemetry board transceiver code
 * @version 0.1
 * @date 2023-03-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "Arduino.h"
#include <SoftwareSerial.h>

#define FREQUENCY_868

#include "LoRa_E32.h"

enum TransceiverState {
    RX, TX
};

struct TelemetryPacket {
    uint32_t timestamp;
    uint8_t state;
    float pressure;
    float temperature;
    uint8_t vBatt;
    uint8_t abPct
    int16_t acX;
    int16_t acY;
    int16_t acZ;
    int16_t gyX;
    int16_t gyY;
    int16_t gyZ;
};

class TelemetryBoard {
public:

    TelemetryBoard();

    bool init();

    void onLoop(uint32_t timestamp);

    void setState(TransceiverState newState);

private:

    constexpr static int PIN_M0 = 2;
    constexpr static int PIN_M1 = 3;
    constexpr static int PIN_AUX = 4;

    constexpr static int CHAN = 58;
    constexpr static int ADDH = 0;
    constexpr static int ADDL = 0;
    
    TransceiverState transmitterState = RX;

    uint8_t packetSize = sizeof(TelemetryPacket);

    LoRa_E32 e32ttl = LoRa_E32(&Serial1, PIN_AUX, PIN_M0, PIN_M1);

    void printParameters(struct Configuration configuration);

    void printModuleInformation(struct ModuleInformation moduleInformation);

    TelemetryPacket transmitPacket;

};
