#include <Wire.h>
#include "SensorBoardLibraries\SensorBoard.hpp"

static const uint32_t LoopFrequency = 100UL; // 100 Hz

static const byte MCP2517FD_CS = 10;
static const byte MCP2517FD_INT = 2;

Sensorboard sensorboard;
// ACAN2517FD can(MCP2517FD_CS, SPI, MCP2517FD_INT);
void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  Wire.begin();
  Wire.setClock(400000);
  if (!sensorboard.setup()) {
    Serial.println("Sensorboard setup failed");
  }
  else{
    Serial.println("Sensorboard setup success");
  }

  SPI.begin();
  // ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_20MHz, 500UL * 1000UL, DataBitRateFactor::x10);
  // settings.mRequestedMode = ACAN2517FDSettings::InternalLoopBack;
  // settings.mDriverTransmitFIFOSize = 3;
  // settings.mDriverReceiveFIFOSize = 3;
  // const uint32_t errorCode = can.begin(settings, [] { can.isr(); });
  // if (errorCode == 0) {
  //   Serial.print ("Bit Rate prescaler: ") ;
  //   Serial.println (settings.mBitRatePrescaler) ;
  //   Serial.print ("Arbitration Phase segment 1: ") ;
  //   Serial.println (settings.mArbitrationPhaseSegment1) ;
  //   Serial.print ("Arbitration Phase segment 2: ") ;
  //   Serial.println (settings.mArbitrationPhaseSegment2) ;
  //   Serial.print ("Arbitration SJW:") ;
  //   Serial.println (settings.mArbitrationSJW) ;
  //   Serial.print ("Actual Arbitration Bit Rate: ") ;
  //   Serial.print (settings.actualArbitrationBitRate ()) ;
  //   Serial.println (" bit/s") ;
  //   Serial.print ("Exact Arbitration Bit Rate ? ") ;
  //   Serial.println (settings.exactArbitrationBitRate () ? "yes" : "no") ;
  //   Serial.print ("Arbitration Sample point: ") ;
  //   Serial.print (settings.arbitrationSamplePointFromBitStart ()) ;
  //   Serial.println ("%") ;
  //   Serial.println("CAN setup success");
  // }else{
  //   Serial.print ("Configuration error 0x") ;
  //   Serial.println (errorCode, HEX) ;
  // }
  sensorboard.setup();
}

static const uint32_t interval = (uint32_t)(1000UL / LoopFrequency);
volatile uint32_t prevTime = 0;

void loop() {
if (millis() - prevTime >= interval){ // Timer
    prevTime = millis(); // Reset timer before reading sensors
    sensorboard.readSensor(); // Read sensors
    // CANFDMessage message; // Create CAN message
    // message.id = 0x0001U; // Set ID
    // message.ext = false; // Set extended ID
    // message.len = 48; // Set length
    // for (int i = 0; i < 48; i++) { // Set data
    //   message.data[i] = sensorboard.getBuffer(i); // Get data from sensorboard
    // }
    // can.tryToSend(message); // Send CAN message
  }
}

