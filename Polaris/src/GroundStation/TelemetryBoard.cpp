/**
 * @file TelemetryBoard.cpp
 * @author Ground Station
 * @brief Telemetry board transceiver code
 * @version 0.1
 * @date 2023-1-20
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

    float pressure = currentRocketPacket.pressure;
    uint8_t * prsB = (uint8_t *) &pressure;

    // float accelx = currentRocketPacket.accelX;
    // uint8_t * acxB = (uint8_t *) &accelx;

    // float accely = currentRocketPacket.accelY;
    // uint8_t * acyB = (uint8_t *) &accely;

    // float accelz = currentRocketPacket.accelZ;
    // uint8_t * aczB = (uint8_t *) &accelz;

    // float gyrox = currentRocketPacket.gyroX;
    // uint8_t * gyxB = (uint8_t *) &gyrox;

    // float gyroy = currentRocketPacket.gyroY;
    // uint8_t * gyyB = (uint8_t *) &gyroy;

    // float gyroz = currentRocketPacket.gyroZ;
    // uint8_t * gyzB = (uint8_t *) &gyroz;

    // Serial.println(sizeof(currentRocketPacket));

    // Serial.print("Timestamp: ");
    // Serial.println(timestamp);

    // Serial.println("----");
    // Serial.print("Altitude: ");
    // Serial.println(altitude);

    // Serial.println("----");
    // Serial.print("Temp: ");
    // Serial.println(temperature);
    // Serial.println("----");

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

    Serial.print(PRESSURE_IDENT);
    Serial.write(prsB[3]);
    Serial.write(prsB[2]);
    Serial.write(prsB[1]);
    Serial.write(prsB[0]);

    // Serial.print(ACCELX_IDENT);
    // Serial.write(acxB[3]);
    // Serial.write(acxB[2]);
    // Serial.write(acxB[1]);
    // Serial.write(acxB[0]);

    // Serial.print(ACCELY_IDENT);
    // Serial.write(acyB[3]);
    // Serial.write(acyB[2]);
    // Serial.write(acyB[1]);
    // Serial.write(acyB[0]);

    // Serial.print(ACCELZ_IDENT);
    // Serial.write(aczB[3]);
    // Serial.write(aczB[2]);
    // Serial.write(aczB[1]);
    // Serial.write(aczB[0]);

    // Serial.print(GYROX_IDENT);
    // Serial.write(gyxB[3]);
    // Serial.write(gyxB[2]);
    // Serial.write(gyxB[1]);
    // Serial.write(gyxB[0]);

    // Serial.print(GYROY_IDENT);
    // Serial.write(gyyB[3]);
    // Serial.write(gyyB[2]);
    // Serial.write(gyyB[1]);
    // Serial.write(gyyB[0]);

    // Serial.print(GYROY_IDENT);
    // Serial.write(gyzB[3]);
    // Serial.write(gyzB[2]);
    // Serial.write(gyzB[1]);
    // Serial.write(gyzB[0]);
    
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

            Serial.println("TX: Low Power!");

            transceiver->SendStruct(&currentRocketPacket, sizeof(currentRocketPacket));
            break;
        }

        case HIGH_POWER: {

            Serial.println("TX: High Power!");

            transceiver->SendStruct(&currentRocketPacket, sizeof(currentRocketPacket));
            break;
        }
    }
};