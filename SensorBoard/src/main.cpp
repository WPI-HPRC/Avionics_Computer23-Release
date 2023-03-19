// #include <Wire.h>
// #include "SensorBoardLibraries\SensorBoard.hpp"

// static const uint32_t LoopFrequency = 1UL; // 1 Hz

// static const byte MCP2517FD_CS = 10;
// static const byte MCP2517FD_INT = 2;

// Sensorboard sensorboard;
// ACAN2517FD can(MCP2517FD_CS, SPI, MCP2517FD_INT);
// void setup() {
//   Serial.begin(115200);
//   while(!Serial) {}
//   Wire.begin();
//   Wire.setClock(400000);
//   if (!sensorboard.setup()) {
//     Serial.println("Sensorboard setup failed");
//   }
//   else{
//     Serial.println("Sensorboard setup success");
//   }
//   SPI.begin () ;
// //--- Configure ACAN2517FD
//   Serial.print ("sizeof (ACAN2517FDSettings): ") ;
//   // Serial.print (sizeof (ACAN2517FDSettings)) ;
//   Serial.println (" bytes") ;
//   Serial.println ("Configure ACAN2517FD") ;
// //--- For version >= 2.1.0
//   ACAN2517FDSettings settings (ACAN2517FDSettings::OSC_20MHz, 125UL * 1000UL, DataBitRateFactor::x1) ;
// //--- For version < 2.1.0
// //  ACAN2517FDSettings settings (ACAN2517FDSettings::OSC_4MHz10xPLL, 125UL * 1000UL, ACAN2517FDSettings::DATA_BITRATE_x1) ;
//  // settings.mRequestedMode = ACAN2517FDSettings::ListenOnly ; // Select loopback mode
// //--- Default values are too high for an Arduino Uno that contains 2048 bytes of RAM: reduce them
//   settings.mDriverTransmitFIFOSize = 1 ;
//   settings.mDriverReceiveFIFOSize = 1 ;
// //--- RAM Usage
//   Serial.print ("MCP2517FD RAM Usage: ") ;
//   Serial.print (settings.ramUsage ()) ;
//   Serial.println (" bytes") ;
// //--- Begin
//   const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
//   if (errorCode == 0) {
//     Serial.print ("Bit Rate prescaler: ") ;
//     Serial.println (settings.mBitRatePrescaler) ;
//     Serial.print ("Arbitration Phase segment 1: ") ;
//     Serial.println (settings.mArbitrationPhaseSegment1) ;
//     Serial.print ("Arbitration Phase segment 2: ") ;
//     Serial.println (settings.mArbitrationPhaseSegment2) ;
//     Serial.print ("Arbitration SJW:") ;
//     Serial.println (settings.mArbitrationSJW) ;
//     Serial.print ("Actual Arbitration Bit Rate: ") ;
//     Serial.print (settings.actualArbitrationBitRate ()) ;
//     Serial.println (" bit/s") ;
//     Serial.print ("Exact Arbitration Bit Rate ? ") ;
//     Serial.println (settings.exactArbitrationBitRate () ? "yes" : "no") ;
//     Serial.print ("Arbitration Sample point: ") ;
//     Serial.print (settings.arbitrationSamplePointFromBitStart ()) ;
//     Serial.println ("%") ;
//   }else{
//     Serial.print ("Configuration error 0x") ;
//     Serial.println (errorCode, HEX) ;
//   }
// }

// // static const uint32_t interval = (uint32_t)(1000UL / LoopFrequency);
// volatile uint32_t prevTime = 0;

// void loop() {
//   Serial.println("Loop");
//     sensorboard.readSensor(); // Read sensors
//     CANFDMessage message; // Create CAN message
//     message.id = 0x0001U; // Set ID
//     message.ext = false; // Set extended ID
//     message.len = 64; // Set length
//     char buffer[64];
//     memcpy(buffer, &sensorboard.frame, 64);
//     for (int i = 0; i < 64; i++) {
//       message.data[i] = buffer[i];
//     }
//     Serial.print("Sending: ");
//     bool ok = can.tryToSend(message); // Send CAN 
//     if (ok) {
//       Serial.println("OK");
//     } else {
//       Serial.println("Failed");
//     }
// }


//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Demo in loopback mode, for Arduino Uno
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2517FD.h>
#include <SPI.h>

//——————————————————————————————————————————————————————————————————————————————
// Very very important: put a 10kΩ resistor between CS and VDD of MCP2517FD

static const byte MCP2517_CS  = 10 ; // CS input of MCP2517
static const byte MCP2517_INT =  2 ; // INT output of MCP2517

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2517FD can (MCP2517_CS, SPI, MCP2517_INT) ;

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup () {
//--- Start serial
  Serial.begin (9600) ;
//--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
  }
//----------------------------------- Begin SPI
  SPI.begin () ;
//--- Configure ACAN2517FD
  Serial.print ("sizeof (ACAN2517FDSettings): ") ;
  // Serial.print (sizeof (ACAN2517FDSettings)) ;
  Serial.println (" bytes") ;
  Serial.println ("Configure ACAN2517FD") ;
//--- For version >= 2.1.0
  ACAN2517FDSettings settings (ACAN2517FDSettings::OSC_20MHz, 125UL * 1000UL, DataBitRateFactor::x1) ;
//--- For version < 2.1.0
//  ACAN2517FDSettings settings (ACAN2517FDSettings::OSC_4MHz10xPLL, 125UL * 1000UL, ACAN2517FDSettings::DATA_BITRATE_x1) ;
 // settings.mRequestedMode = ACAN2517FDSettings::ListenOnly ; // Select loopback mode
//--- Default values are too high for an Arduino Uno that contains 2048 bytes of RAM: reduce them
  settings.mDriverTransmitFIFOSize = 1 ;
  settings.mDriverReceiveFIFOSize = 1 ;
//--- RAM Usage
  Serial.print ("MCP2517FD RAM Usage: ") ;
  Serial.print (settings.ramUsage ()) ;
  Serial.println (" bytes") ;
//--- Begin
  const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Arbitration Phase segment 1: ") ;
    Serial.println (settings.mArbitrationPhaseSegment1) ;
    Serial.print ("Arbitration Phase segment 2: ") ;
    Serial.println (settings.mArbitrationPhaseSegment2) ;
    Serial.print ("Arbitration SJW:") ;
    Serial.println (settings.mArbitrationSJW) ;
    Serial.print ("Actual Arbitration Bit Rate: ") ;
    Serial.print (settings.actualArbitrationBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact Arbitration Bit Rate ? ") ;
    Serial.println (settings.exactArbitrationBitRate () ? "yes" : "no") ;
    Serial.print ("Arbitration Sample point: ") ;
    Serial.print (settings.arbitrationSamplePointFromBitStart ()) ;
    Serial.println ("%") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————
//   LOOP
//——————————————————————————————————————————————————————————————————————————————

static uint32_t gSendDate = 0 ;
static uint32_t gReceiveDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

//——————————————————————————————————————————————————————————————————————————————

void loop () {
  CANFDMessage frame ;
  frame.len = 8;
 for(int i = 0 ; i < 8;i++){
  if(i%2 ==0){
     frame.data[i] = 69;
  }else{
    frame.data[i] = 0b10101010;
  }
   
 }
  if (gSendDate < millis ()) {
    gSendDate += 1000 ;
    const bool ok = can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.print (gSentFrameCount) ;
    }else{
      Serial.print ("Send failure") ;
    }
    Serial.print (", receive overflows: ") ;
    Serial.println (can.hardwareReceiveBufferOverflowCount ()) ;
  }
//  if (gReceiveDate < millis ()) {
//    gReceiveDate += 1000 ;
//    while (can.available ()) {
//      can.receive (frame) ;
//      gReceivedFrameCount ++ ;
//      Serial.print ("Received: ") ;
//      for(int i = 0 ; i < 8;i++){
//        Serial.print(frame.data[i]);
//      }
//      Serial.println (gReceivedFrameCount) ;
//    }
//  }
}

//——————————————————————————————————————————————————————————————————————————————