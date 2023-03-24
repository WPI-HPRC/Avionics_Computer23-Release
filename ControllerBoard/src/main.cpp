#include <Arduino.h>
#include "Libraries/MetroTimer/Metro.h"
#include "ControllerBoardConstants.h"
#include "Libraries/ACAN2517FD/ACAN2517FD.h"

// CAN Setup
static const byte MCP2518FD_CS = 10;
static const byte MCP2518FD_INT = 2;

ACAN2517FD can(MCP2518FD_CS, SPI, MCP2518FD_INT);

Metro timer = Metro(CONVERSION / LOOP_FREQUENCY); // Hz converted to ms
int counter = 0;                                  // counts how many times the loop runs
float timestamp;

Metro prelaunchTimer = Metro(PRELAUNCH_INTERVAL);
Metro burnoutTimer = Metro(MOTOR_BURNOUT_INTERVAL); // 1 second timer after motor burnout detected
Metro boostTimer = Metro(BOOST_MIN_LENGTH);
boolean burnoutDetected = false;

int16_t transitionBuf[10];
uint8_t transitionBufInd = 0;

boolean secondSweep = false;

int16_t accelMag;

unsigned long state_start;

enum AvionicsState
{
    STARTUP,
    PRELAUNCH,
    BOOST,
    COAST_CONTINGENCY,
    COAST,
    DROGUE_DEPLOY,
    DROGUE_CONTINGENCY,
    DROGUE_DESCENT,
    MAIN_CONTINGENCY,
    MAIN_DEPLOY,
    MAIN_DESCENT,
    POSTFLIGHT,
    ABORT
};

AvionicsState avionicsState = ABORT;

// checks whether the state has timed out yet
bool timeout(uint16_t length)
{
    // check state length against current time - start time for the current state
    if (millis() - state_start >= length)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// bool updateSensorData() {
//   // process sensor data when it comes in over CAN, put in data fields
//   // Read CAN messages
// }

// uses updated accel value to determine if the rocket has launched
// stores 10 most recent values and computes current avg
boolean launchDetect()
{
    // accel value gets updated in sensor reading fcn
    // add to cyclic buffer
    transitionBuf[transitionBufInd] = accelMag;
    // take running average value
    float sum = 0.0;
    for (int i = 0; i < 10; i++)
    {
        sum += transitionBuf[i];
    }
    sum = sum / 10.0;
    transitionBufInd = (transitionBufInd + 1) % 10;
    // compare running average value to defined threshold
    if (sum > ACCEL_THRESHOLD)
    {
        for (int j = 0; j < 10; j++)
        {
            transitionBuf[j] = 0;
        }
        transitionBufInd = 0;
        Serial.println("Launch detected!");
        return true;
    }
    return false;
}

// detects if motor burnout has occurred (aka negative accel)
// tracks 10 most recent accels and computes current average
boolean motorBurnoutDetect()
{
    // accel value gets updated in sensor reading fcn
    // add to cyclic buffer
    transitionBuf[transitionBufInd] = accelMag;
    // take running average value
    float sum = 0.0;
    for (int i = 0; i < 10; i++)
    {
        sum += transitionBuf[i];
    }
    sum = sum / 10.0;

    transitionBufInd = (transitionBufInd + 1) % 10;
    // compare running average value to defined threshold
    if (sum < 0)
    {
        for (int j = 0; j < 10; j++)
        {
            transitionBuf[j] = 0;
        }
        transitionBufInd = 0;
        Serial.println("Motor burnout detected!");
        return true;
    }
    return false;
}

boolean apogeeDetect()
{
    // if descending, case = DESCENT
    // Decreasing altitude
    // add to cyclic buffer
    transitionBuf[transitionBufInd] = accelMag;
    // not going to be running avg, so this needs to be changed
    float sum = 0.0;
    for (int i = 0; i < 10; i++)
    {
        sum += transitionBuf[i];
    }
    sum = sum / 10.0;
    transitionBufInd = (transitionBufInd + 1) % 10;
    //   if (sum < 0) {
    //	   Serial.println("Apogee detected!");
    //     return true;
    //   }
    return false;
}

boolean landingDetect()
{
    return false; // TODO: WRITE SOMETHING HERE FOR REAL
}

void readSensors()
{
    // TODO: Populate these from Sensor Board
    accelMag = 0;
}

void setup()
{
    Serial.begin(9600);
    SPI.begin();
    // put your setup code here, to run once:
    // CAN Setup

    ACAN2517FDSettings settings(
        ACAN2517FDSettings::OSC_20MHz, // oscillator
        500UL * 1000UL,                // arbitration bit rate
        DataBitRateFactor::x10         // bit rate factor
    );
    settings.mRequestedMode = ACAN2517FDSettings::InternalLoopBack;
    settings.mDriverTransmitFIFOSize = 3;
    settings.mDriverReceiveFIFOSize = 10;

    const uint32_t errorCode = can.begin(settings, []
                                         { can.isr(); });
    if (errorCode == 0)
    {
        Serial.print("Bit Rate prescaler: ");
        Serial.println(settings.mBitRatePrescaler);
        Serial.print("Arbitration Phase segment 1: ");
        Serial.println(settings.mArbitrationPhaseSegment1);
        Serial.print("Arbitration Phase segment 2: ");
        Serial.println(settings.mArbitrationPhaseSegment2);
        Serial.print("Arbitration SJW:");
        Serial.println(settings.mArbitrationSJW);
        Serial.print("Actual Arbitration Bit Rate: ");
        Serial.print(settings.actualArbitrationBitRate());
        Serial.println(" bit/s");
        Serial.print("Exact Arbitration Bit Rate ? ");
        Serial.println(settings.exactArbitrationBitRate() ? "yes" : "no");
        Serial.print("Arbitration Sample point: ");
        Serial.print(settings.arbitrationSamplePointFromBitStart());
        Serial.println("%");
        Serial.println("CAN setup success");
    }
    else
    {
        Serial.print("Configuration error 0x");
        Serial.println(errorCode, HEX); // getting error code 0x1 kRequestedConfigurationModeTimeOut
    }
}

void loop()
{
    // put your main code here, to run repeatedly:
    if (timer.check() == 1)
    { // controls how frequently this loop runs
        // update function call, receives CAN messages
        // updateSensorData();

        // TODO write ABORT state switches
        switch (avionicsState)
        {

        case STARTUP:
            if (prelaunchTimer.check() == 1) // 1 second timeout
            {
                avionicsState = PRELAUNCH;
                state_start = millis();
            }

            break;
        case PRELAUNCH:

            // if accel high enough, case = BOOST
            if (launchDetect())
            {
                avionicsState = BOOST;
                boostTimer.reset();
                state_start = millis();
            }
            break;
        case BOOST:
            // Stay in this state for at least 3 seconds to prevent airbrake activation
            if (boostTimer.check() == 1)
            {
                if (motorBurnoutDetect() && !burnoutDetected)
                {
                    burnoutDetected = true;
                    burnoutTimer.reset();
                }

                if (burnoutDetected)
                {
                    // airbrakeServo.enable;  // Add this back in when using the stack
                    state_start = millis();
                    avionicsState = COAST;
                    // airbrakesTimer.reset();
                    break;
                }
            }
            // Coast Contingency 8 second timeout
            if (timeout(8000))
            {
                Serial.println("Boost phase timeout triggered!");
                state_start = millis();
                avionicsState = COAST_CONTINGENCY;
                break;
            }
            break;
        case COAST_CONTINGENCY:
            // TODO Check if Coast appears normal, then move into COAST phase
            // state_start = millis();
            // avionicsState = COAST;
            // break;

            // TODO Determine timeout length
            if (timeout(10000))
            {
                Serial.println("Coast Contingency phase timeout triggered!");
                state_start = millis();
                avionicsState = DROGUE_CONTINGENCY;
                break;
            }
            break;
        case COAST:
            // Wait 30 seconds before moving into DROGUE_DEPLOY state
            if (timeout(30000))
            {
                Serial.println("Coast phase timeout triggered!");
                state_start = millis();
                avionicsState = DROGUE_DEPLOY;
                break;
            }

            // active airbrakes control
            // Sweep program for test launch
            // Send message to power board to actuate the motor
            // airbrakeSweep();

            // if descending, case = DESCENT
            // Velocity 0?
            // Decreasing altitude

            if (apogeeDetect())
            {
                // airbrakeServo.setPosition(0); // Need to do this upon state transition
                // TODO: Add some delay on a timer to ensure airbrakes get fully retracted
                // airbrakeServo.disable;  // Add this back in upon state transition when using the stack
                state_start = millis();
                avionicsState = DROGUE_DEPLOY;
            }

            break;
        case DROGUE_DEPLOY:

            break;
        case DROGUE_CONTINGENCY:
            // TODO Check for expected drogue descent rate, then move into DROGUE_DESCENT state
            // state_start = millis();
            // avionicsState = DROGUE_DESCENT;
            // break;

            // TODO Determine timeout length
            if (timeout(10000))
            {
                Serial.println("Drogue Contingency phase timeout triggered!");
                state_start = millis();
                avionicsState = MAIN_CONTINGENCY;
                break;
            }
            break;
        case DROGUE_DESCENT:
            if (timeout(100000))
            {
                Serial.println("Drogue Descent phase timeout triggered!");
                avionicsState = MAIN_DEPLOY;
                break;
            }
            break;
        case MAIN_DEPLOY:

            break;
        case MAIN_CONTINGENCY:
            // TODO Check for expected main descent rate, then move into MAIN_DESCENT state
            // state_start = millis();
            // avionicsState = MAIN_DESCENT;
            // break;

            // TODO Determine timeout length
            if (timeout(10000))
            {
                Serial.println("Main Contingency phase timeout triggered!");
                state_start = millis();
                avionicsState = POSTFLIGHT;
                break;
            }
            break;
        case MAIN_DESCENT:
            if (timeout(90000))
            {
                Serial.println("Main Descent phase timeout triggered!");
                avionicsState = POSTFLIGHT;
                break;
            }

            // if accel jumps aka landing, case = POSTFLIGHT
            // What is accel jump? What value are we expecting?
            if (landingDetect())
            {
                avionicsState = POSTFLIGHT;
            }
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

        // May pull these into every state
        // Read all
        // readSensors();

        // Record sensor data to onboard flash chip
        // logData();

        // Transmit data to ground station
        // sendTelemetry();

        counter++;
        timestamp = counter * (CONVERSION / LOOP_FREQUENCY); // TODO CHANGE THIS TO RECORD SYSTEM TIME
        timer.reset();

        // if (counter % 25 == 0) {
        // 	Serial.print("Current state: ");
        // 	Serial.print(avionicsState);
        // 	Serial.print("	Accel val: ");
        // 	Serial.println(accelMag);
        // }
    }
}