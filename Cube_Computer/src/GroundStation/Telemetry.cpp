#include "Telemetry.h"

TelemetryBoard::TelemetryBoard() {
    radio = new LoRaClass();
}

void TelemetryBoard::init() {

    radio->setPins(LORA_CS);

    if(radio->begin(915E6)) {
        Serial.println("[Radio] Successfully Initialized");
    } else {
        Serial.println("[Radio] Failed to Initialize");
    }
}

void TelemetryBoard::setState(TransceiverState newState) {
    this->transmitterState = newState;
}

void TelemetryBoard::onLoop(TelemetryPacket telemPacket) {
    switch(transmitterState) {
        case(TX): {
            transmitPacket = telemPacket;
            uint8_t txBuffer[sizeof(transmitPacket)];
            memcpy(txBuffer, &transmitPacket, sizeof(transmitPacket));

            String structString = telemPacket.name + "," + String(millis()) + "," + String(telemPacket.temperature) + "," + String(telemPacket.pressure) + "," + String(telemPacket.humidity);

            radio->beginPacket();
            // radio->write(txBuffer, sizeof(telemPacket));
            radio->print(structString);
            radio->endPacket();
            Serial.print("Packet ("); Serial.print(millis()); Serial.println(")");

            break;
        }
        case (RX): {
            TelemetryPacket rxPacket;
            radio->readBytes((uint8_t *) &rxPacket, sizeof(rxPacket));

            break;
        }
    }
}

void TelemetryBoard::printPacketToGS(TelemetryPacket rxPacket) {
    uint32_t timestamp = rxPacket.timestamp;
    uint8_t * tspB = (uint8_t *) &timestamp;
}