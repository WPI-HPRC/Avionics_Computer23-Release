#include <Arduino.h>
#include "Libraries/MetroTimer/Metro.h"
#include "ControllerBoardConstants.h"
#include "Libraries/ACAN2517FD/ACAN2517FD.h"
#include "ControllerBoardLibraries/Sensor_Frames.hpp"
#include "ControllerBoardLibraries/TelemetryFrame.hpp"
#include "Controller.h"
#include "ControllerBoardLibraries/StateEstimator.h"

// CAN Setup
static const byte MCP2518FD_CS = 10;
static const byte MCP2518FD_INT = 2;
ACAN2517FD can(MCP2518FD_CS, SPI, MCP2518FD_INT);

// Airbrake controller class
Controller controller;

// State estimation class
StateEstimator stateEstimator;

// Main timer declarations
Metro timer = Metro(CONVERSION / LOOP_FREQUENCY); // Hz converted to ms
int counter = 0;                                  // counts how many times the loop runs
unsigned long state_start;                        // must be set to millis() when entering a new state

Metro prelaunchTimer = Metro(PRELAUNCH_INTERVAL);   // 1 second timer to stay in STARTUP before moving to PRELAUNCH
Metro boostTimer = Metro(BOOST_MIN_LENGTH);         // 3 second timer to ensure BOOST state is locked before possibility of state change

// Declarations for state transition detection buffer
int16_t transitionBuf[10];
uint8_t transitionBufInd = 0;

// Declaration for sensor data struct
SensorFrame sensorPacket;

// Declaration for GPS data struct
GPSFrame gpsPacket;

// Declaration for telemetry packet struct
TelemetryFrame telemPacket;

// Declaration for state estimate struct
StateStruct stateStruct;

// Variable declarations for measured values
uint8_t vBatt;

// Variable declarations for filtered state data
float altitude;
int16_t posX;
int16_t posY;
int16_t posZ;
int16_t velX;
int16_t velY;
int16_t velZ;
int16_t vel_vert;   // temporary??
int16_t vel_lat;    // temporary??
int16_t vel_total;  // temporary??
uint8_t abPct;

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

AvionicsState avionicsState = STARTUP;

// checks whether the state has timed out yet
bool timeout(uint32_t length)
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

// uses updated accel value to determine if the rocket has launched
// stores 10 most recent values and computes current avg
boolean launchDetect()
{
    // accel value gets updated in sensor reading fcn
    // add to cyclic buffer
    transitionBuf[transitionBufInd] = sensorPacket.Z_accel;
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
    transitionBuf[transitionBufInd] = sensorPacket.Z_accel;
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
    transitionBuf[transitionBufInd] = sensorPacket.Z_accel;
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

// Compute altitude in [m] from pressure in [mBar]
float pressureToAltitude(float pressure_mBar) {

    // physical parameters for model
    const float pb = 101325;    // [Pa] pressure at sea level
    const float Tb = 288.15;    // [K] temperature at seal level
    const float Lb = -0.0065;   // [K/m] standard temperature lapse rate
    const float hb = 0;         // [m] height at bottom of atmospheric layer (sea level)
    const float R = 8.31432;    // [N*m/mol*K] universal gas constant
    const float g0 = 9.80665;   // [m/s^2] Earth standard gravity
    const float M = 0.0289644;  // [kg/mol] molar mass of Earth's air

    // convert pressure from [mBar] to [Pa]
    float pressure_Pa = pressure_mBar*100;

    // compute altitude from formula
    return hb + (Tb/Lb) * (pow((pressure_Pa/pb), (-R * Lb / (g0 * M))) - 1);
}

// Construct the data packet
void constructPacket() {

    // TODO: When state estiation is completed, need to update this to include velocity, orientation, etc.

    // Timestamp
    telemPacket.timestamp = sensorPacket.time;

    // State
    telemPacket.state = (uint8_t) avionicsState;

    // Altitude
    telemPacket.altitude = altitude;

    // Temperature
    telemPacket.temperature = sensorPacket.Temperature;

    // Battery voltage
    telemPacket.vBatt = (uint8_t) vBatt*20;

    // Airbrake actuation percent
    telemPacket.abPct = abPct;

    // Acceleration (XYZ)
    telemPacket.ac_x = (int16_t) sensorPacket.X_accel*100;
    telemPacket.ac_y = (int16_t) sensorPacket.Y_accel*100;
    telemPacket.ac_z = (int16_t) sensorPacket.Z_accel*100;

    // Angular rate (XYZ)
    telemPacket.gy_x = (int16_t) sensorPacket.X_gyro*10;
    telemPacket.gy_y = (int16_t) sensorPacket.Y_gyro*10;
    telemPacket.gy_z = (int16_t) sensorPacket.Z_gyro*10;

    // Velocity (vertical, lateral, total)
    telemPacket.vel_vert = (int16_t) vel_vert;
    telemPacket.vel_lat = (int16_t) vel_lat;
    telemPacket.vel_total = (int16_t) vel_total;
}

// Process incoming messages from the CAN bus
void retrieveCAN()
{
    // TODO: Read messages while CAN available. Write if statements to unpack data dependent upon message id.
    CANFDMessage message;

    // Unpack data frame from sensor board
    if (message.id == CAN_ID_SENSOR_DATA) {    
        memcpy(&sensorPacket, message.data, 48);
        altitude = pressureToAltitude(sensorPacket.Pressure);
    }
    
    // Unpack GPS frame from sensor board
    else if (message.id == CAN_ID_GPS) {
        memcpy(&gpsPacket, message.data, 24);
    }

    // Unpack battery voltage from power board
    else if (message.id == CAN_ID_VBATT) {
        vBatt = message.data[0];
    }

    // Construct data packet
    constructPacket();
}

// Write data (telemPacket) to flash chip
void logData()
{
    // TODO: Write data packet to flash chip
}

// Perform state estimation for the launch vehicle
//  Inputs: raw sensor data (currently altitude + XYZ acceleration)
//  Output: stateStruct (currently vertical, lateral, and total veloctity)
void doStateEstimation() {
    stateStruct = stateEstimator.getState(altitude, sensorPacket.X_accel, sensorPacket.Y_accel, sensorPacket.Z_accel);
}

// Compute airbrake actuation percent from controller.
//      Inputs: vertical velocity, lateral velocity, altitude
//      Output: airbrake actuation level as a percent from 0 to 100
void doAirbrakeControls() {
    abPct = controller.calcAbPct(stateStruct.vel_vert, stateStruct.vel_lat, altitude);
}

// Send a message to place devices into low power mode and possibly decrease datalogging rate
void lowPowerMode() {
    // TODO: Do something. Probably want to send a state variable to all boards eventually.
}

// Send CAN frame with data packet
void sendCAN() {
    CANFDMessage message;
    message.id = CAN_ID_DATA_PACKET;
    message.len = 24;
    memcpy(message.data, &telemPacket, 24);
    const bool ok = can.tryToSend (message) ;
    if (ok) { // TODO: Print statements here are for debugging... could actually handle a failure somehow if we wanted to
      // Serial.print ("Sent: ") ;
    } else {
      // Serial.print ("Send failure") ;
    }
}

// Built-in Arduino setup function. Performs initilization tasks on startup.
void setup()
{
    // Communications setup
    Serial.begin(9600);
    SPI.begin();

    // TODO: Write a function to send a command to fully retract airbrakes, wait 1 second, then send servo disable command

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
        Serial.println(errorCode, HEX);
    }
}

// Built-in Arduino loop function. Executes main control loop at a specified frequency.
void loop()
{
    // controls how frequently this loop runs, defined by LOOP_FREQUENCY in the constants file
    if (timer.check() == 1)
    {
        // update function call, receives CAN messages
        // updateSensorData();

        switch (avionicsState)
        {

        case STARTUP:
            // STARTUP state must be active for at least 1 second
            if (prelaunchTimer.check() == 1)
            {
                avionicsState = PRELAUNCH;
                state_start = millis();
            }
            break;
        case PRELAUNCH:
            // detect acceleration of 3G's
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
                if (motorBurnoutDetect())
                {
                    // airbrakeServo.enable;
                    state_start = millis();
                    avionicsState = COAST;
                    break;
                }
            }
            // Coast Contingency 8 second timeout
            // Burnout wasn't detected after 8 seconds, move into coast contingency
            // if (timeout(BOOST_TIMEOUT))
            // {
            //     Serial.println("Boost phase timeout triggered!");
            //     state_start = millis();
            //     avionicsState = COAST_CONTINGENCY;
            //     break;
            // }

            // ABORT Case
            // Pitch or roll exceeds 30 degrees from vertical
            // Acceleration is greater than 10g's

            // break;
        // case COAST_CONTINGENCY:
        //     // TODO Check if Coast appears normal, then move into COAST phase
        //     // state_start = millis();
        //     // avionicsState = COAST;
        //     // break;

        //     // TODO Determine timeout length
        //     if (timeout(10000))
        //     {
        //         Serial.println("Coast Contingency phase timeout triggered!");
        //         state_start = millis();
        //         avionicsState = DROGUE_CONTINGENCY;
        //         break;
        //     }
        //     break;
        case COAST:

            doAirbrakeControls();

            // TODO: flesh this out more, need to clarify apogee detection
            if (apogeeDetect())
            {
                abPct = 0; // Retract airbrakes fully upon apogee detecetion
                // TODO: Add some delay on a timer to ensure airbrakes get fully retracted
                // airbrakeServo.disable;
                state_start = millis();
                avionicsState = DROGUE_DEPLOY;
            }

            // Wait 30 seconds before moving into DROGUE_DEPLOY state if all else fails
            if (timeout(COAST_TIMEOUT))
            {
                Serial.println("Coast phase timeout triggered!");
                state_start = millis();
                avionicsState = DROGUE_DEPLOY;
                break;
            }

            // ABORT Case
            // Pitch or roll exceeds 30 degrees from vertical
            // Acceleration is greater than 10g's
                // 10-15 second timeout on 10g check to ensure ejection load doesn't accidentally trigger ABORT

            break;
        case DROGUE_DEPLOY:
            // TODO: Check for nominal drogue descent rate, then move into DROGUE_DESCENT state
            // Poll for a second?
            // state_start = millis();
            // avionicsState = DROGUE_DESCENT;

            // ABORT Case
            // 10 second timeout
            if (timeout(DROGUE_DEPLOY_TIMEOUT)) {
                state_start = millis();
                avionicsState = ABORT;
            }
            break;
        // case DROGUE_CONTINGENCY:
        //     // TODO Check for expected drogue descent rate, then move into DROGUE_DESCENT state
        //     // state_start = millis();
        //     // avionicsState = DROGUE_DESCENT;
        //     // break;

        //     // TODO Determine timeout length
        //     if (timeout(10000))
        //     {
        //         Serial.println("Drogue Contingency phase timeout triggered!");
        //         state_start = millis();
        //         avionicsState = MAIN_CONTINGENCY;
        //         break;
        //     }
        //     break;
        case DROGUE_DESCENT:
            // detect altitude drop below 1500ft
            // state_start = millis();
            // avionicsState = MAIN_DEPLOY;

            // // 120 second contingency timeout
            // if (timeout(DROGUE_DESCENT_TIMEOUT)) {
            //     Serial.println("Drogue Descent phase timeout triggered!");
            //     state_start = millis();
            //     avionicsState = MAIN_CONTINGENCY;
            //     break;
            // }
            break;
        case MAIN_DEPLOY:
            // TODO: Check for nominal main descent rate, then move into MAIN_DESCENT state
            // Poll for a second?
            // state_start = millis();
            // avionicsState = MAIN_DESCENT;

            // ABORT Case
            // 10 second timeout
            if (timeout(MAIN_DEPLOY_TIMEOUT)) {
                state_start = millis();
                avionicsState = ABORT;
            }
            break;
        // case MAIN_CONTINGENCY:
        //     // TODO Check for expected main descent rate, then move into MAIN_DESCENT state
        //     // state_start = millis();
        //     // avionicsState = MAIN_DESCENT;
        //     // break;

        //     // TODO Determine timeout length
        //     if (timeout(10000))
        //     {
        //         Serial.println("Main Contingency phase timeout triggered!");
        //         state_start = millis();
        //         avionicsState = POSTFLIGHT;
        //         break;
        //     }
        //     break;
        case MAIN_DESCENT:
            // transmit data during this phase

            // altitude below 50ft
            // other checks?
            if (landingDetect())
            {
                state_start = millis();
                avionicsState = POSTFLIGHT;
            }

            // timeout after 100 seconds if other checks fail
            if (timeout(MAIN_DESCENT_TIMEOUT))
            {
                Serial.println("Main Descent phase timeout triggered!");
                avionicsState = POSTFLIGHT;
                break;
            }
            break;
        case POSTFLIGHT:
            // datalog slow
            lowPowerMode();
            break;
        case ABORT:
            // jump to here if anything goes terribly wrong (but obv it won't)
            // retract airbrakes
            // stop all other processes
            // Start sending high-fidelity data backwards in time from abort trigger to launch from flash
            break;

        default:
            break;
        }

        // Receive inbound CAN frames
        retrieveCAN();

        // Send outbound CAN frames
        sendCAN();

        counter++;
        timer.reset();
    }
}
