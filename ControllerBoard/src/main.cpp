#include <Arduino.h>

#define ACCEL_THRESHOLD 10

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
}

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:

  // update function call
  updateSensorData();

  switch (state) {

    case PRELAUNCH:
      // init sensors, datalog

      // if accel high enough, case = BOOST
      if (accelMag > ACCEL_THRESHOLD) {
        state = BOOST;
      }

      break;
    case BOOST:
      // current est = 5.5 sec
      if (timeout(6)) {
        state = ABORT;
        break;
      }
      // datalog

      // if decel occurs, case = COAST
      if (accelMag < ACCEL_THRESHOLD) { // or more sophisticated test for decel?
        state = COAST;
      }

      break;
    case COAST:
      // current est = 20 sec
      if (timeout(20)) {
        state = ABORT;
        break;
      }
      // active airbrakes control

      // if descending, case = DESCENT

      break;
    case DESCENT:
      // current est = 100 (drogue) + 90 (main)
      if (timeout(200)) {
        state = ABORT;
        break;
      }
      // datalog

      // if accel jumps aka landing, case = POSTFLIGHT

      break;
    case POSTFLIGHT:
      // datalog

      break;
    case ABORT:
      // jump to here if anything goes terribly wrong (but obv it won't)

      break;

    default:

      break;
  }
}