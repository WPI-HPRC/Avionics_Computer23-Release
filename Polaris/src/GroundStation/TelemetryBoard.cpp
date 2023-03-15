#include "TelemetryBoard.h"

TelemetryBoard::TelemetryBoard(BoardType boardType) {
    this->boardType = boardType;
}

int TelemetryBoard::init() {
    switch(boardType) {
        case(teensy): { // If teensy, initialize hw serial 1 and begin at 9600 baud
            Serial1.begin(9600);
            Serial.println("Teensy");

            transceiver = new LoRaE32(&Serial1, PIN_M0, PIN_M1, PIN_AUX);
            break;
        }
        case(atmega): { //If atmega, initialize software serial and begin at 9600baud
            SoftwareSerial ESerial(PIN_RX, PIN_TX);
            ESerial.begin(9600);

            transceiver = new LoRaE32(&ESerial, PIN_M0, PIN_M1, PIN_AUX);
            break;
        }
    }

    if(!transceiver->init()) {
        Serial.println("Error initializing telemetry!");
        return -1;
    }

    // Configure EByte E32-900T20S
    transceiver->SetAddressH(0);
    transceiver->SetAddressL(0); // Broadcast and rx all
    transceiver->SetChannel(58); // 58 - 920MHz for E32-900T20S
                                 // 20 - 920MHz for E32-915T20D
    
    transceiver->SetParityBit(0); // Parity bit -> 8N1
    transceiver->SetUARTBaudRate(3); // 3 = 9600baud
    transceiver->SetAirDataRate(4); // 3 = 4.8kbps, 4 = 9.6kbps

    transceiver->SetTransmissionMode(0);
    transceiver->SetPullupMode(1);
    transceiver->SetWORTIming(0);
    transceiver->SetFECMode(1);
    transceiver->SetTransmitPower(0); // Max Power

    transceiver->SaveParameters(PERMANENT);
    transceiver->PrintParameters();

    return 1;
}

int TelemetryBoard::onLoop() {

    switch(telemetryState) {
        case(RX): {
            // Serial.println(transceiver->available());
            if(transceiver->available()) {
                transceiver->GetStruct(&currentRocketPacket, packetSize);
                printPacketToGS();
            }

            break;
        }
        case(TX): {
            bool packetSuccess = transceiver->SendStruct(&currentRocketPacket, sizeof(currentRocketPacket));
            Serial.print("Packet: ");
            Serial.println(currentRocketPacket.timestamp);
            Serial.print("Success?: ");
            Serial.println(packetSuccess);
            break;
        }
    }

    return 1;
}

void TelemetryBoard::printPacketToGS() {
    uint32_t timestamp = currentRocketPacket.timestamp;
    uint8_t * tspB = (uint8_t *) &timestamp;

    uint8_t state = currentRocketPacket.state;

    float temperature = currentRocketPacket.temperature;
    uint8_t * tmpB = (uint8_t *) &temperature;

    float pressure = currentRocketPacket.pressure;
    uint8_t * prsB = (uint8_t *) &pressure;

    Serial.print(PACKET_BEG);
    
    Serial.print(TIMESTAMP_IDENT);
    Serial.write(tspB[3]);
    Serial.write(tspB[2]);
    Serial.write(tspB[1]);
    Serial.write(tspB[0]);

    Serial.print(STATE_IDENT);
    Serial.write(state);

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

    Serial.print(PACKET_END);
}

//Getters
TelemBoardState TelemetryBoard::getState() {
    return telemetryState;
}

//Setters
void TelemetryBoard::setState(TelemBoardState state) {
    this->telemetryState = state;
}

void TelemetryBoard::setCurrentPacket(RocketPacket newPacket) {
    // this->currentRocketPacket = newPacket;
    memcpy(&currentRocketPacket, &newPacket, packetSize);
}