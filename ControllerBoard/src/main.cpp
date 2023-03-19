// LoopBackDemo for Teensy 4.x CAN3 in CANFD mode

// The FlexCAN module is configured in loop back mode:
//   it internally receives every CAN frame it sends.

// No external hardware required.

//-----------------------------------------------------------------

#ifndef __IMXRT1062__
  #error "This sketch should be compiled for Teensy 4.x"
#endif

//-----------------------------------------------------------------

#include <ACAN_T4.h>
#include "SensorboardFrame.hpp"

//-----------------------------------------------------------------

void setup () {
  Serial.begin (115200) ;
  while (!Serial) {
    delay (50) ;
  }
  Serial.println ("CAN3FD loopback test") ;
  ACAN_T4FD_Settings settings (125 * 1000, DataBitRateFactor::x4) ;
  settings.mLoopBackMode = true ;
  settings.mSelfReceptionMode = true ;
  const uint32_t errorCode = ACAN_T4::can3.beginFD (settings) ;
  if (0 == errorCode) {
    Serial.println ("can3 ok") ;
  }else{
    Serial.print ("Error can3: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//-----------------------------------------------------------------

static uint32_t gBlinkDate = 0 ;
static uint32_t gSendDate = 0 ;
static uint32_t gSentCount = 0 ;
static uint32_t gReceivedCount = 0 ;

//-----------------------------------------------------------------

void loop () {
  CANFDMessage message;
  SensorboardFrame sensorboardFrame;
  if (ACAN_T4::can3.receiveFD (message)) {
    gReceivedCount += 1;
    Serial.print ("Received: ") ;
    Serial.println (gReceivedCount) ;
    char buffer [64] ;
    memcpy (buffer, message.data, message.len) ;
    memcpy (&sensorboardFrame, buffer, 64) ;
    Serial.println("SensorboardFrame: ");
    Serial.print ("  ");
    Serial.print("accelerationX: ");
    Serial.print(sensorboardFrame.X_accel);
    Serial.print("  ");
    Serial.print("accelerationY: ");
    Serial.print(sensorboardFrame.Y_accel);
    Serial.print("  ");
    Serial.print("accelerationZ: ");
    Serial.print(sensorboardFrame.Z_accel);
  }

}

//-----------------------------------------------------------------