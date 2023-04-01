/**
 * @file TelemetryBoard.h
 * @author Daniel Pearson
 * @brief Telemetry board transceiver code
 * @version 0.1
 * @date 2023-03-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>

#define FREQUENCY_868

#include <GroundStation/LoRa_E32.h>

enum TelemBoardState {
    RX, TX
};

struct TelemetryPacket {
    // byte timestamp[4];
    // byte state[1];
    // byte altitude[4];
    // byte temperature[1];
    // byte vBat[1];
    // byte abPct[1];
    // byte acx[2];
    // byte acy[2];
    // byte acz[2];
    // byte gyx[2];
    // byte gyy[2];
    // byte gyz[2];
    
    uint32_t timestamp;
    uint8_t state;
    float altitude;
    int8_t temperature;
    uint8_t vBatt;
    uint8_t abPct;
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

    void setState(TelemBoardState newState);
private:
    constexpr static int PIN_M0 = 2;
    constexpr static int PIN_M1 = 3;
    constexpr static int PIN_AUX = A3;
    constexpr static int PIN_RX = A2;
    constexpr static int PIN_TX = 4;

    constexpr static int CHAN = 58;
    constexpr static int ADDH = 0;
    constexpr static int ADDL = 0;

    TelemBoardState telemetryState = RX;

    uint8_t packetSize = sizeof(TelemetryPacket);

    TelemetryPacket txPacket;

    LoRa_E32 e32ttl = LoRa_E32(PIN_TX, PIN_RX, PIN_AUX, PIN_M0, PIN_M1);

    void printParameters(struct Configuration configuration);

    void printModuleInformation(struct ModuleInformation moduleInformation);

    void setPacket(TelemetryPacket updatedTxPacket);
};