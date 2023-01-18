/**
 * @file TelemetryBoard.h
 * @author Ground Station
 * @brief Telemetry board transceiver header
 * @version 0.1
 * @date 2023-1-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "Arduino.h"
#include "LoRaE32.h"

#include <SoftwareSerial.h>

#define PACKET_BEG "BEGB"
#define TIMESTAMP_IDENT "TSP"
#define STATE_IDENT "STT"
#define ALTITUDE_IDENT "ALT"
#define TEMPERATURE_IDENT "TMP"
#define PACKET_END "ENDB"

enum TelemBoardState {
    RX, LOW_POWER, HIGH_POWER
};

class TelemetryBoard {

public:
    TelemetryBoard();

    void setState(TelemBoardState state);
    void printToGS();
    int initTelem();
    void onLoop();
    TelemBoardState getState();

    void setTimestamp(uint32_t currentTimestamp);
    void setTelemState(uint8_t currentState);
    void setAltitude(float currrentAltitude);
    void setTemperature(float currentTemperature);

private:
    constexpr static uint8_t PIN_M0  = 2;
	constexpr static uint8_t PIN_M1  = 3;
	constexpr static uint8_t PIN_AUX = 4;
	constexpr static uint8_t DATA_SIZE = 32; // Bytes
    constexpr static byte rxPin = 4;
    constexpr static byte txPin = 5;

    struct RocketPacket {
        uint32_t timestamp;
        uint8_t state;
        float altitude;
        float temperature;
    };

    TelemBoardState telemetryState = RX;

    uint8_t currentPacket[DATA_SIZE];
    RocketPacket currentRocketPacket;
    SoftwareSerial ESerial = SoftwareSerial(rxPin, txPin);

    LoRaE32 * transceiver = new LoRaE32(&ESerial, PIN_M0, PIN_M1, PIN_AUX);

    // LoRaE32 * transceiver = new LoRaE32(&Serial1, PIN_M0, PIN_M1, PIN_AUX);

    // LoRaE32 * transceiver = new LoRaE32(&LoRaSerial, PIN_M0, PIN_M1, PIN_AUX);
    
};