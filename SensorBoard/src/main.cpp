#include <ACAN2517FD.h>
#include <SPI.h>
#include <Wire.h>
#include <SensorBoardLibraries\Sensorboard.hpp>

static const byte MCP2517_CS  = 10 ; // CS input of MCP2517FD
static const byte MCP2517_INT = 2 ; // INT output of MCP2517FD

// Intialize the CAN Controller and the Sensorboard
ACAN2517FD can (MCP2517_CS, SPI, MCP2517_INT); 
Sensorboard sensorboard;

static const uint32_t Loop_Frequency = 100; // Hz
static const uint32_t Loop_Period = 1000 / Loop_Frequency; // ms

void setup () {
  Serial.begin (115200); // Initialize the serial port
  while (!Serial) { // Wait for the serial port to be opened
    delay (5) ;
  }
  // Intialize Wire for I2C communication
  Wire.begin();
  Wire.setClock(400000);

  // Initialize/Setup the CAN Controller
  SPI.begin () ;
  ACAN2517FDSettings settings (ACAN2517FDSettings::OSC_20MHz, 125UL * 1000UL, DataBitRateFactor::x8) ;

  settings.mRequestedMode = ACAN2517FDSettings::InternalLoopBack ; // Select loopback mode

  settings.mDriverTransmitFIFOSize = 3 ;
  settings.mDriverReceiveFIFOSize = 3 ;

  const uint32_t errorCode = can.begin (settings, []{can.isr();}) ; 
  if (errorCode == 0) {
    // Serial.println ("ACAN2517FD successfully configured") ;
    // Serial.print ("Bit Rate prescaler: ") ;
    // Serial.println (settings.mBitRatePrescaler) ;
    // Serial.print ("Arbitration Phase segment 1: ") ;
    // Serial.println (settings.mArbitrationPhaseSegment1) ;
    // Serial.print ("Arbitration Phase segment 2: ") ;
    // Serial.println (settings.mArbitrationPhaseSegment2) ;
    // Serial.print ("Arbitration SJW:") ;
    // Serial.println (settings.mArbitrationSJW) ;
    // Serial.print ("Actual Arbitration Bit Rate: ") ;
    // Serial.print (settings.actualArbitrationBitRate ()) ;
    // Serial.println (" bit/s") ;
    // Serial.print ("Exact Arbitration Bit Rate ? ") ;
    // Serial.println (settings.exactArbitrationBitRate () ? "yes" : "no") ;
    // Serial.print ("Arbitration Sample point: ") ;
    // Serial.print (settings.arbitrationSamplePointFromBitStart ()) ;
    // Serial.println ("%") ;
  }else{
    // Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }

  // Initialize/Setup the Sensorboard
  if(sensorboard.setup()){
    // Serial.println("Sensorboard initialized");
  }else{
    // Serial.println("Sensorboard initialization failed");
  }
}


uint32_t prevTime = 0;
void loop () {
  if (millis() - prevTime >= Loop_Period) {
    prevTime = millis();
    sensorboard.readSensor();
    CANFDMessage frame;
    frame.id = 0x01;
    frame.ext = 0;
    frame.len = 64;
    memcpy(frame.data, &sensorboard.frame,64);
    const bool ok = can.tryToSend (frame);
    // if (ok) {
    //   Serial.println("Sent");
    // }else{
    //   Serial.println("Send failure");
    // }
  }

}