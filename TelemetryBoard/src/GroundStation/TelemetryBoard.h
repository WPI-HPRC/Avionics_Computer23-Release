/**
 * @file TelemetryBoard.h
 * @author Ground Station
 * @brief Telemetry board transceiver code
 * @version V2
 * @date 2023-2-5
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "LoRaE32.h"

#define PACKET_BEG "BEGB"
#define TIMESTAMP_IDENT "TSP"
#define STATE_IDENT "STT"
#define ALTITUDE_IDENT "ALT"
#define TEMPERATURE_IDENT "TMP"
#define PRESSURE_IDENT "PRR"
#define PACKET_END "ENDB"

enum TelemBoardState {
    RX, TX
};

enum BoardType {
    teensy, atmega
};

struct RocketPacket {
    uint32_t timestamp;
    uint32_t state;
    float pressure;
    float temperature;
};

class TelemetryBoard {
public:
    TelemetryBoard(); //Constructor

    int init(); // Initalize transmitter

    void printPacketToGS(); //prints Current packet to ground station

    int onLoop(); // Run once per loop cycle

    //Getters
    TelemBoardState getState();

    //Setters
    void setState(TelemBoardState state); //State switcher
    
        //Telemetry
    void setCurrentPacket(RocketPacket newPacket);

private:
    constexpr static uint8_t PIN_M0 = 2;
    constexpr static uint8_t PIN_M1 = 3;
    constexpr static uint8_t PIN_AUX = A3;
    
    //Serial RX and TX are only used when not using teensy hw serial 1
    constexpr static uint8_t PIN_RX = 4;
    constexpr static uint8_t PIN_TX = A2;

    TelemBoardState telemetryState = RX;

    RocketPacket currentRocketPacket;
    uint8_t packetSize = sizeof(currentRocketPacket);

    LoRaE32 * transceiver;
    SoftwareSerial * ESerial = new SoftwareSerial(PIN_RX, PIN_TX);
};