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

#include <GroundStation/LoRa_E32.h>

enum TransceiverState {
    RX, TX
};

struct TelemetryPacket {
    uint32_t timestamp; // System time from power board, running from startup onward. Given in milliseconds.
    uint8_t state;      // Rocket mission state from state machine.
    uint8_t vBatt;      // Battery voltage. Given in Volts. Scaled by 10.
    float altitude;     // Converted altitude measurement. Given in meters.
    int8_t temperature; // Temperature measurement. Given in degrees Celsius.
    uint8_t abPct;      // Airbrake actuation level. Given as percentage from 0 to 100.
    int16_t ac_x;       // X-axis acceleration. Given in m/s^2. Scaled by 100.
    int16_t ac_y;       // Y-axis acceleration. Given in m/s^2. Scaled by 100.
    int16_t ac_z;       // Z-axis acceleration. Given in m/s^2. Scaled by 100.
    int16_t gy_x;       // Angular rotation rate about X-axis. Given in degrees/s. Scaled by 10.
    int16_t gy_y;       // Angular rotation rate about Y-axis. Given in degrees/s. Scaled by 10.
    int16_t gy_z;       // Angular rotation rate about Z-axis. Given in degrees/s. Scaled by 10.
    int16_t vel_vert;   // Vertical velocity. Given in m/s. 
    int16_t vel_lat;    // Vertical velocity. Given in m/s. 
    int16_t vel_total;  // Vertical velocity. Given in m/s. 
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

    void onLoop(TelemetryPacket telemPacke);

    void setState(TransceiverState newState);

    TransceiverState getState() {
        return transmitterState;
    }

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

    void printPacketToGS(TelemetryPacket rxPacket);

    TelemetryPacket transmitPacket;

};
