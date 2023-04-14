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
    radio.setPacketType(PACKET_TYPE_LORA);
    radio.setRfFrequency(frequency, offset); // Set Frequency to 920MHz
    radio.calibrateImage(0); // Calibrate Transceiver
    radio.setModulationParams(SpreadingFactor, bandwith, CodeRate, LDRO_AUTO);
    radio.setBufferBaseAddress(0x00, 0x00);
    radio.setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
    radio.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);
    radio.setHighSensitivity();
    radio.setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);

    Serial.println();
    radio.printModemSettings();
    Serial.println();
    radio.printOperatingSettings();
    Serial.println();
    Serial.println();
    radio.printRegisters(0x00, 0x4F);  
    Serial.println();
    Serial.println();

    Serial.print(F("Transmitter ready"));

}

void Telemetry::onLoop(uint32_t timestamp) {
    uint8_t tempPacket[] = "Hello World";
    packetLen = sizeof(tempPacket);
    tempPacket[packetLen -1] = '*';

    radio.printASCIIPacket(tempPacket, packetLen);
    
    uint8_t packetStatus = radio.transmit(tempPacket, packetLen, 10000, TXPower, WAIT_TX);
    Serial.println("Packet Status: " + String(packetStatus));
    Serial.print("TSP: "); Serial.println(timestamp);
}

void Telemetry::setState(TransceiverState state) {
    // Do nothing
}