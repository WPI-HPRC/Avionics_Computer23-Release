/**
 * @file TelemetryBoard.h
 * @author Ground Station
 * @brief Telemetry board transceiver code
 * @version 0.1
 * @date 2022-12-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "Arduino.h"
#include "../EmbeddedSystems/LoRaE32.h"

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

    struct RocketPacket {
        uint32_t timestamp;
        uint8_t state;
        float altitude;
        float temperature;
        int16_t accelX;
        int16_t accelY;
        int16_t accelZ;
    };

    TelemBoardState getState() {
        return this->telemetryState;
    }

    void setRocketPacket(RocketPacket newPacket) {
        this->currentRocketPacket = newPacket;
    }

private:
    TelemBoardState telemState = RX;

    constexpr static uint8_t PIN_M0  = 2;
	constexpr static uint8_t PIN_M1  = 3;
	constexpr static uint8_t PIN_AUX = 4;
	constexpr static uint8_t DATA_SIZE = 32; // Bytes

    TelemBoardState telemetryState = RX;

    uint8_t currentPacket[DATA_SIZE];
    RocketPacket currentRocketPacket;

    LoRaE32 * transceiver = new LoRaE32(&Serial1, PIN_M0, PIN_M1, PIN_AUX);



    // void setTelemState(uint8_t currentState) {
    //     this->currentRocketPacket.state = currentState;
    // }

    // void setTemperature(float currentTemperature)  {
    //     this->currentRocketPacket.temperature = currentTemperature;
    // }

    // void setTimestamp(uint32_t currentTimestamp) {
    //     this->currentRocketPacket.timestamp = currentTimestamp;
    // }

    // void setAltitude(float currentAltitude) {
    //     this->currentRocketPacket.altitude = currentAltitude;
    // }
    
};