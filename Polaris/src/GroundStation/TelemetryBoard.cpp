#include <GroundStation/TelemetryBoard.h>

TelemetryBoard::TelemetryBoard() {
    e32ttl.begin();
}

bool TelemetryBoard::init() {
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

void TelemetryBoard::onLoop(uint32_t timestamp) {
    switch(transmitterState) {
        case(TX): {
            transmitPacket.timestamp = timestamp;
            transmitPacket.state = 0;
            transmitPacket.temperature = 32.0;
            transmitPacket.pressure = 29.92;
            transmitPacket.acX = 10;
            transmitPacket.acY = 10;
            transmitPacket.acZ = 10;
            transmitPacket.gyX = 10;
            transmitPacket.gyY = 10;
            transmitPacket.gyZ = 10;

            ResponseStatus rs = e32ttl.sendFixedMessage(ADDH,ADDL,CHAN, &transmitPacket, sizeof(TelemetryPacket));

            Serial.print("Packet ("); Serial.print(timestamp); Serial.print("): "); Serial.println(rs.getResponseDescription());

            break;
        }
        case (RX): {
            if(e32ttl.available() > 1) {
                ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(TelemetryPacket));
                TelemetryPacket newPacket = *(TelemetryPacket*) rsc.data;
                Serial.print("Timestamp: "); Serial.println(newPacket.timestamp);

                rsc.close();
                
            }

            break;
        }
    }
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