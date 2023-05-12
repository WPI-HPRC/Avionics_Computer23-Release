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

#include <LoRa.h>

enum TransceiverState {
    RX, TX
};

struct TelemetryPacket {
    String name; // Name of the packet, used for identification
    uint32_t timestamp; // System time from power board, running from startup onward. Given in milliseconds.
    float pressure;
    float humidity;
    float temperature;
};

#define PACKET_BEG "BEGB"
#define TIMESTAMP_IDENT "TSP"
#define STATE_IDENT "STT"
#define ALTITUDE_IDENT "ALT"
#define TEMPERATURE_IDENT "TMP"
#define VOLTAGE_IDENT "VLT"
#define AIRBRAKES_IDENT "ARB"
#define PACKET_END "ENDB"


class TelemetryBoard {
public:

    TelemetryBoard();

    void init();

    void onLoop(TelemetryPacket telemPacket);

    void setState(TransceiverState newState);

    TransceiverState getState() {
        return transmitterState;
    }

private:
    constexpr static int LORA_CS = 6;
    
    TransceiverState transmitterState = RX;

    uint8_t packetSize = sizeof(TelemetryPacket);

    void printPacketToGS(TelemetryPacket rxPacket);

    TelemetryPacket transmitPacket;

    LoRaClass * radio;

};