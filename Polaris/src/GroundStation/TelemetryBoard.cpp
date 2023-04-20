#include <GroundStation/TelemetryBoard.h>

TelemetryBoard::TelemetryBoard() {
    e32ttl.begin();
}

void TelemetryBoard::init() {
    ResponseStructContainer c;
    c = e32ttl.getConfiguration();
    Configuration config = *(Configuration*) c.data;

    config.ADDL = ADDL;
    config.ADDH = ADDH;
    config.CHAN = CHAN;

    config.SPED.airDataRate = 5;
    config.SPED.uartBaudRate = 3;
    config.SPED.uartParity = 0;

    config.OPTION.fec = 1;
    config.OPTION.transmissionPower = 0;
    config.OPTION.wirelessWakeupTime = 0;
    config.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;

    e32ttl.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
    printParameters(config);

    c.close();
}

void TelemetryBoard::setState(TransceiverState newState) {
    this->transmitterState = newState;
}

void TelemetryBoard::onLoop(TelemetryPacket telemPacket) {
    switch(transmitterState) {
        case(TX): {
            transmitPacket = telemPacket;

            ResponseStatus rs = e32ttl.sendFixedMessage(ADDH,ADDL,CHAN, &transmitPacket, packetSize);

            Serial.print("Packet ("); Serial.print(transmitPacket.timestamp); Serial.print("): "); Serial.println(rs.getResponseDescription());

            break;
        }
        case (RX): {
            if(e32ttl.available() > 1) {
                ResponseStructContainer rsc = e32ttl.receiveMessage(packetSize);
                TelemetryPacket newPacket = *(TelemetryPacket*) rsc.data;
                printPacketToGS(newPacket);

                rsc.close();
                
            }

            break;
        }
    }
}

void TelemetryBoard::printPacketToGS(TelemetryPacket rxPacket) {
    uint32_t timestamp = rxPacket.timestamp;
    uint8_t * tspB = (uint8_t *) &timestamp;

    uint8_t state = rxPacket.state;

    float altitude = rxPacket.altitude;
    uint8_t * altB = (uint8_t *) &altitude;

    float accelX = (float) rxPacket.ac_x / 100.0;
    uint8_t * acxB = (uint8_t *) &accelX;

    float accelY = (float) rxPacket.ac_y / 100.0;
    uint8_t * acyB = (uint8_t *) &accelY;
    
    float accelZ = (float) rxPacket.ac_z / 100.0;
    uint8_t * aczB = (uint8_t *) &accelZ;

    // Serial.print("ACX: "); Serial.println(accelZ);
    // Serial.print("ACX: "); Serial.println(rxPacket.ac_z);

    float gyroX = (float) rxPacket.gy_x / 10.0;
    uint8_t * gyX = (uint8_t *) &gyroX;

    float gyroY = (float) rxPacket.gy_x / 10.0;
    uint8_t * gyY = (uint8_t *) &gyroY;
    
    float gyroZ = (float) rxPacket.gy_z / 10.0;
    uint8_t * gyZ = (uint8_t *) &gyroZ;

    int16_t totalVel = rxPacket.vel_total / 100.0;
    uint8_t * tvB = (uint8_t *) &totalVel;

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
    Serial.write(rxPacket.temperature);

    Serial.print(AIRBRAKES_IDENT);
    Serial.write(rxPacket.abPct);

    Serial.print("ACX");
    Serial.write(acxB[3]);
    Serial.write(acxB[2]);
    Serial.write(acxB[1]);
    Serial.write(acxB[0]);

    Serial.print("ACY");
    Serial.write(acyB[3]);
    Serial.write(acyB[2]);
    Serial.write(acyB[1]);
    Serial.write(acyB[0]);

    Serial.print("ACZ");
    Serial.write(aczB[3]);
    Serial.write(aczB[2]);
    Serial.write(aczB[1]);
    Serial.write(aczB[0]);

    Serial.print("GYX");
    Serial.write(gyX[3]);
    Serial.write(gyX[2]);
    Serial.write(gyX[1]);
    Serial.write(gyX[0]);

    Serial.print("GYY");
    Serial.write(gyY[3]);
    Serial.write(gyY[2]);
    Serial.write(gyY[1]);
    Serial.write(gyY[0]);
    
    Serial.print("GYZ");
    Serial.write(gyZ[3]);
    Serial.write(gyZ[2]);
    Serial.write(gyZ[1]);
    Serial.write(gyZ[0]);

    Serial.print("VEL");
    Serial.write(tvB[1]);
    Serial.write(tvB[0]);

    Serial.print(PACKET_END);
}

void TelemetryBoard::printParameters(struct Configuration configuration) {
    Serial.println("----------------------------------------");

	Serial.print(F("HEAD : "));  Serial.print(configuration.HEAD, BIN);Serial.print(" ");Serial.print(configuration.HEAD, DEC);Serial.print(" ");Serial.println(configuration.HEAD, HEX);
	Serial.println(F(" "));
	Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, DEC);
	Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, DEC);
	Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
	Serial.println(F(" "));
	Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
	Serial.print(F("SpeedUARTDatte  : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRate());
	Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRate());

	Serial.print(F("OptionTrans        : "));  Serial.print(configuration.OPTION.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFixedTransmissionDescription());
	Serial.print(F("OptionPullup       : "));  Serial.print(configuration.OPTION.ioDriveMode, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getIODroveModeDescription());
	Serial.print(F("OptionWakeup       : "));  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
	Serial.print(F("OptionFEC          : "));  Serial.print(configuration.OPTION.fec, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFECDescription());
	Serial.print(F("OptionPower        : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());
	Serial.println("----------------------------------------");

}

void TelemetryBoard::printModuleInformation(struct ModuleInformation moduleInformation) {
    Serial.println("----------------------------------------");
	Serial.print(F("HEAD BIN: "));  Serial.print(moduleInformation.HEAD, BIN);Serial.print(" ");Serial.print(moduleInformation.HEAD, DEC);Serial.print(" ");Serial.println(moduleInformation.HEAD, HEX);

	Serial.print(F("Freq.: "));  Serial.println(moduleInformation.frequency, HEX);
	Serial.print(F("Version  : "));  Serial.println(moduleInformation.version, HEX);
	Serial.print(F("Features : "));  Serial.println(moduleInformation.features, HEX);
	Serial.println("----------------------------------------");
}