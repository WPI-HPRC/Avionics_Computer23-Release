#include <Arduino.h>
#include "Libraries/ACAN2517FD/ACAN2517FD.h"
#include "Libraries/MetroTimer/Metro.h"
#include "ControllerBoardConstants.h"
#include "Airbrakes.h"

// CAN Setup
static const byte MCP2517FD_CS = 10;
static const byte MCP2517FD_INT = 2;

ACAN2517FD can(MCP2517FD_CS, SPI, MCP2517FD_INT);
Metro timer = Metro(1000UL / LOOP_FREQUENCY); // Hz converted to ms
int counter = 0; // counts how many times the loop runs
int timestamp;

int16_t accelMag;

enum AvionicsState {
  PRELAUNCH,
  BOOST,
  COAST,
  DESCENT,
  POSTFLIGHT,
  ABORT
};

AvionicsState state = ABORT;

// checks whether the state has timed out yet
bool timeout(uint16_t length) {
  // check state length against current time - start time for the current state
  return false;
}

bool updateSensorData() {
  // process sensor data when it comes in over CAN, put in data fields
  // Read CAN messages
}

void setup() {
  // put your setup code here, to run once:
  // CAN Setup cont.
  ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_20MHz, 500UL * 1000UL, DataBitRateFactor::x10);
  settings.mRequestedMode = ACAN2517FDSettings::InternalLoopBack;
  settings.mDriverTransmitFIFOSize = 3;
  settings.mDriverReceiveFIFOSize = 10;
  const uint32_t errorCode = can.begin(settings, [] { can.isr(); });
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
    Serial.println("CAN setup success");
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (timer.check() == 1) { // controls how frequently this loop runs
    // update function call, receives CAN messages
    // updateSensorData();

    switch (state) {

      case PRELAUNCH:
        // init sensors, datalog

        // Close airbrakes

        // if accel high enough, case = BOOST
        if (accelMag > ACCEL_THRESHOLD) {
          state = BOOST;
        }

        break;
      case BOOST:
        // current est = 5.5 sec
        if (timeout(6)) { // Will this timeout hold us in the state? Need to globalize this timeout somehow
          state = ABORT;
          break;
        }
        // datalog

        // Keep airbrakes closed

        // if decel occurs, case = COAST
        // Check if acceleration is around a threshold of 0
          // How accurate is the Kalman filter?

        break;
      case COAST:
        // current est = 20 sec
        if (timeout(20)) {
          state = ABORT;
          break;
        }
        // active airbrakes control
        // Sweep program for test launch

        // if descending, case = DESCENT
          // Velocity 0?
          // Decreasing altitude

        break;
      case DESCENT:
        // current est = 100 (drogue) + 90 (main)
        if (timeout(200)) {
          state = ABORT;
          break;
        }
        // datalog

        // if accel jumps aka landing, case = POSTFLIGHT
          // What is accel jump? What value are we expecting?

        break;
      case POSTFLIGHT:
        // datalog

        break;
      case ABORT:
        // jump to here if anything goes terribly wrong (but obv it won't)
        // datalog

        break;

      default:

        break;
    }
    counter++;
    timestamp = counter * (1000UL / LOOP_FREQUENCY);
    // Send CAN messages
    
  }
}