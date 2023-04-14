#include "Telemetry.h"

Telemetry::Telemetry() {
    // Do nothing
    
}

void Telemetry::init() {
    if(!radio.begin(LORA_CS, LORA_RST, DEVICE_SX1276)) {
        Serial.println("SX1276 not found");
    } else {
        Serial.println("SX1276 detected");
    }
    radio.setMode(MODE_STDBY_RC); // Set transceiver mode to standby for configuration
    radio.setPacketType(PACKET_TYPE_GFSK);
    radio.setRfFrequency(frequency, offset); // Set Frequency to 920MHz
    radio.calibrateImage(0); // Calibrate Transceiver
    
}

void Telemetry::onLoop(uint32_t timestamp) {
    // Do nothing
}

void Telemetry::setState(TransceiverState state) {
    // Do nothing
}