/**
 * @file TelemetryBoard.cpp
 * @author Ground Station
 * @brief Telemetry board transceiver code
 * @version 0.1
 * @date 2023-1-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "TelemetryBoard.h"

TelemetryBoard::TelemetryBoard() {

}

int TelemetryBoard::initTelem() {

    Serial1.begin(9600);

    transceiver->init(1);

    transceiver->SetAddressH(0);
	transceiver->SetAddressL(0);        
    transceiver->SetChannel(58); // this is 920 mHz :)

    transceiver->SetParityBit(0); //Speed bit
    transceiver->SetUARTBaudRate(3); //9600 baud
    transceiver->SetAirDataRate(4); // air data rate: 9.6K ;)

    transceiver->SetTransmissionMode(0);
    transceiver->SetPullupMode(1);
    transceiver->SetWORTIming(1);
    transceiver->SetTransmitPower(0);
    transceiver->SetAddressH(0);
    transceiver->SetAddressL(0);
    transceiver->SetParityBit(0);
    transceiver->SetUARTBaudRate(3);
    transceiver->SetTransmissionMode(0);
    transceiver->SetPullupMode(1);
    transceiver->SetWORTIming(1);

    transceiver->SaveParameters(); // NOTIHNG WILL BE SAVED... UNLESS... SAVEPARAMETERS IS CALLED :(
    transceiver->PrintParameters();

    Serial.println("Telemetry board intialized!");

    return 1;
}

void TelemetryBoard::setState(TelemBoardState state) {
    this->telemetryState = state;
}

void TelemetryBoard::printToGS() {
    uint32_t timestamp = currentRocketPacket.timestamp;
    uint8_t * tspB = (uint8_t *) &timestamp;

    uint8_t state = currentRocketPacket.state;

    float altitude = currentRocketPacket.altitude;
    uint8_t * altB = (uint8_t *) &altitude;

    float temperature = currentRocketPacket.temperature;
    uint8_t * tmpB = (uint8_t *) &temperature;

    Serial.print(PACKET_BEG);
    
    Serial.print(TIMESTAMP_IDENT);
    Serial.write(tspB[3]);
    Serial.write(tspB[2]);
    Serial.write(tspB[1]);
    Serial.write(tspB[0]);

    Serial.print(STATE_IDENT);
    Serial.write(state);

    Serial.print(ALTITUDE_IDENT);
    Serial.write(altB[3]);
    Serial.write(altB[2]);
    Serial.write(altB[1]);
    Serial.write(altB[0]);

    Serial.print(TEMPERATURE_IDENT);
    Serial.write(tmpB[3]);
    Serial.write(tmpB[2]);
    Serial.write(tmpB[1]);
    Serial.write(tmpB[0]);

    Serial.print(PACKET_END);
}

void TelemetryBoard::onLoop() {
    //Update packet

    switch(telemetryState) {
        case RX: {
            if(transceiver->available()) {
                transceiver->GetStruct(&currentRocketPacket, sizeof(currentRocketPacket));
                printToGS();
            }
            break;

        }

        case LOW_POWER: {
            
            transceiver->SetTransmitPower(3);
            transceiver->SaveParameters();

            Serial.println("TX: Low Power!");

            transceiver->SendStruct(&currentRocketPacket, sizeof(currentRocketPacket));
            break;
        }

        case HIGH_POWER: {

            Serial.println("TX: High Power!)");

            transceiver->SendStruct(&currentRocketPacket, sizeof(currentRocketPacket));
            break;
        }
    }
};