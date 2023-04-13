#include <Arduino.h>
#include "lib/ServoDriver/ServoDriver.h"
#include "lib/MetroTimer/Metro.h"
#include "ControllerBoardLibraries/ControllerBoardConstants.h"
#include "ControllerBoardLibraries/Controller.h"
#include "ControllerBoardLibraries/StateEstimator.h"
#include "lib/Flash/Flash.h"
#include "SensorBoardLibraries/SensorBoard.hpp"
#include "GroundStation/TelemetryBoard.h"

// Airbrake controller class
Controller controller;

// State estimation class
StateEstimator stateEstimator;

// Flash chip instantiation
FlashChip flash = FlashChip();

// Telemetry Board class
TelemetryBoard telemBoard = TelemetryBoard();

// Sensor Board class
Sensorboard sensorboard;

// Airbrakes servo
ServoDriver airbrakeServo(SERVO_PIN); // Instantiate airbrake servo

// Main timer declarations
Metro timer = Metro(CONVERSION / LOOP_FREQUENCY); // Hz converted to ms
int counter = 0;                                  // counts how many times the loop runs
unsigned long state_start;                        // must be set to millis() when entering a new state

Metro prelaunchTimer = Metro(PRELAUNCH_INTERVAL); // 1 second timer to stay in STARTUP before moving to PRELAUNCH
Metro boostTimer = Metro(BOOST_MIN_LENGTH);       // 3 second timer to ensure BOOST state is locked before possibility of state change

// Declarations for state transition detection buffer
int16_t transitionBuf[10];
uint8_t transitionBufInd = 0;

// Declaration for sensor data struct, measured values
SensorFrame sensorPacket;

// Declaration for GPS data struct
GPSFrame gpsPacket;

// Declaration for telemetry packet struct, calculated values
TelemetryPacket telemPacket;

// Declaration for state estimate struct
StateStruct stateStruct;

// Variable declarations for filtered state data
float altitude;
int16_t vel_vert;
int16_t vel_lat;
int16_t vel_total;
int16_t ac_total;
uint8_t abPct;

int16_t sumMainDescentVel = 0;
int16_t mainPollCount = 0;

int16_t sumDrogueDescentVel = 0;
int16_t droguePollCount = 0;

// Enumeration for rocket mission states
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
    transitionBuf[transitionBufInd] = sensorPacket.ac_z;
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
    transitionBuf[transitionBufInd] = sensorPacket.ac_z;
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

// TODO THIS REALLY NEEDS TO BE DONE
boolean apogeeDetect()
{
    // if descending, case = DESCENT
    // Decreasing altitude
    // add to cyclic buffer
    transitionBuf[transitionBufInd] = sensorPacket.ac_z;
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

// Convert pressure in [mBar] to altitude in [m]
float pressureToAltitude(float pressure_mBar)
{

    // physical parameters for model
    const float pb = 101325;   // [Pa] pressure at sea level
    const float Tb = 288.15;   // [K] temperature at seal level
    const float Lb = -0.0065;  // [K/m] standard temperature lapse rate
    const float hb = 0;        // [m] height at bottom of atmospheric layer (sea level)
    const float R = 8.31432;   // [N*m/mol*K] universal gas constant
    const float g0 = 9.80665;  // [m/s^2] Earth standard gravity
    const float M = 0.0289644; // [kg/mol] molar mass of Earth's air

    // convert pressure from [mBar] to [Pa]
    float pressure_Pa = pressure_mBar * 100;

    // compute altitude from formula
    return hb + (Tb / Lb) * (pow((pressure_Pa / pb), (-R * Lb / (g0 * M))) - 1);
}

// Construct the telemetry data packet. This is the struct that gets logged to the flash chip and transmitted to the ground station.
void constructTelemPacket()
{

    // Timestamp
    telemPacket.timestamp = millis(); // TODO: Maybe change this to be time after launch detect?

    // State
    telemPacket.state = (uint8_t)avionicsState;

    // Altitude [m]
    telemPacket.altitude = altitude;

    // Temperature
    telemPacket.temperature = sensorPacket.Temperature;

    // Airbrake actuation percent
    telemPacket.abPct = abPct;

    // Acceleration (XYZ) [m/s^2] - Scaled by 100x for transmission
    telemPacket.ac_x = (int16_t)sensorPacket.ac_x * 100;
    telemPacket.ac_y = (int16_t)sensorPacket.ac_y * 100;
    telemPacket.ac_z = (int16_t)sensorPacket.ac_z * 100;
    telemPacket.ac_total = ac_total * 100;

    // Angular rate (XYZ) [deg/s] - Scaled by 10x for transmission
    telemPacket.gy_x = (int16_t)sensorPacket.gy_x * 10;
    telemPacket.gy_y = (int16_t)sensorPacket.gy_y * 10;
    telemPacket.gy_z = (int16_t)sensorPacket.gy_z * 10;

    // Velocity (vertical, lateral, total) [m/s]
    telemPacket.vel_vert = (int16_t)vel_vert;
    telemPacket.vel_lat = (int16_t)vel_lat;
    telemPacket.vel_total = (int16_t)vel_total;

    // TODO: Add GPS to telemPacket
}

// Read sensors and construct telemPacket
void readSensors()
{
    // Construct sensorPacket struct with raw data from inertial sensors + barometer
    sensorboard.readInertialSensors();
    memcpy(&sensorPacket, &sensorboard.Inertial_Baro_frame, sizeof(sensorboard.Inertial_Baro_frame));

    // Calculate total acceleration
    ac_total = (int16_t)sqrt((sensorPacket.ac_x * sensorPacket.ac_x + sensorPacket.ac_y * sensorPacket.ac_y + sensorPacket.ac_z * sensorPacket.ac_z));

    // Compute altitude from pressure
    altitude = pressureToAltitude(sensorPacket.Pressure);

    // Construct gpsPacket struct with data from GPS
    // TODO: Get GPS data from sensor board class

    // Construct telemetry data packet
    constructTelemPacket();
}

// Write data (telemPacket) to flash chip
void logData()
{
    flash.writeStruct(&telemPacket, sizeof(telemPacket));
}

void sendTelemetry()
{
    telemBoard.onLoop(telemPacket);
}

// Perform state estimation for the launch vehicle
//  Inputs: raw sensor data (currently altitude + total acceleration)
//  Output: stateStruct (currently vertical, lateral, and total veloctity)
void doStateEstimation()
{
    stateStruct = stateEstimator.getState(altitude, ac_total);
}

// Compute airbrake actuation percent from controller class.
//  Inputs: vertical velocity, lateral velocity, altitude
//  Output: airbrake actuation level as a percent from 0 to 100
void doAirbrakeControls()
{
    abPct = controller.calcAbPct(altitude, stateStruct.vel_vert, stateStruct.vel_lat);
    airbrakeServo.setPosition(abPct);
}

// Send a message to place devices into low power mode and possibly decrease datalogging rate
void lowPowerMode()
{
    // TODO: Do something. Probably want to send a state variable to all boards eventually.
}

// Print telemPacket to Serial monitor for debugging purposes
void debugPrint()
{
    Serial.print("Timestamp: ");
    Serial.print(telemPacket.timestamp);
    Serial.println(" ms");
    Serial.print("State: ");
    Serial.println(telemPacket.state);
    Serial.print("Altitude: ");
    Serial.print(telemPacket.altitude);
    Serial.println(" m");
    Serial.print("Temperature: ");
    Serial.print(telemPacket.temperature);
    Serial.println(" degrees C");
    Serial.print("abPct: ");
    Serial.print(telemPacket.abPct);
    Serial.println("%");
    Serial.print("Accel-X: ");
    Serial.print(telemPacket.ac_x / 100.0);
    Serial.println(" m/s^2");
    Serial.print("Accel-Y: ");
    Serial.print(telemPacket.ac_y / 100.0);
    Serial.println(" m/s^2");
    Serial.print("Accel-Z: ");
    Serial.print(telemPacket.ac_z / 100.0);
    Serial.println(" m/s^2");
    Serial.print("Gyro-X: ");
    Serial.print(telemPacket.gy_x / 10.0);
    Serial.println(" degrees/s");
    Serial.print("Gyro-Y: ");
    Serial.print(telemPacket.gy_y / 10.0);
    Serial.println(" degrees/s");
    Serial.print("Gyro-Z: ");
    Serial.print(telemPacket.gy_z / 10.0);
    Serial.println(" degrees/s");
    Serial.print("Vertical velocity: ");
    Serial.print(telemPacket.vel_vert);
    Serial.println(" m/s");
    Serial.print("Lateral velocity: ");
    Serial.print(telemPacket.vel_lat);
    Serial.println(" m/s");
    Serial.print("Total velocity: ");
    Serial.print(telemPacket.vel_total);
    Serial.println(" m/s");
    Serial.println("");
}

// Built-in Arduino setup function. Performs initilization tasks on startup.
void setup()
{
    // Communications setup
    Serial.begin(9600);

    // Telemetry initialization
    telemBoard.setState(RX);
    telemBoard.init();

    // Flash memory initialization
    // flash.init();
    // flash.initialWrite();

    // Sensor initialization
    if (sensorboard.setup())
    {
        Serial.println("Sensor setup sucess!");
    }
    else
    {
        Serial.println("Sensor setup failed.");
    }

    // Initialize airbrake servo and set to fully retracted position
    airbrakeServo.init();
    airbrakeServo.setPosition(0);

    // TODO: Write a function to get initial pressure/altitude on pad? Could use for AGL compensation on altitude
}

// Built-in Arduino loop function. Executes main control loop at a specified frequency.
void loop()
{
    // controls how frequently this loop runs, defined by LOOP_FREQUENCY in the constants file
    if (timer.check() == 1)
    {

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
            if ((telemPacket.ac_total > ((10 * G) * 100)) || (telemPacket.vel_lat / telemPacket.vel_vert > PITCH_FRACTION))
            {
                state_start = millis();
                avionicsState = COAST;
                break;
            }

            break;
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

            if (apogeeDetect())
            {
                airbrakeServo.setPosition(0); // Retract airbrakes fully upon apogee detecetion
                // TODO: Add some delay on a timer to ensure airbrakes get fully retracted
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
            if ((telemPacket.vel_lat / telemPacket.vel_vert) > PITCH_FRACTION)
            {
                state_start = millis();
                avionicsState = ABORT;
                break;
            }

            // Acceleration is greater than 10g's
            // 10-15 second timeout on 10g check to ensure ejection load doesn't accidentally trigger ABORT
            if (ac_total > ((10 * G) * 100))
            {
                if (timeout(13000)) // currently 13 seconds
                {
                    Serial.println("Abort triggered by acceleration greater than 10g's");
                    state_start = millis();
                    avionicsState = ABORT;
                    break;
                }
            }
            break;
        case DROGUE_DEPLOY:
            // TODO: Check for nominal drogue descent rate, then move into DROGUE_DESCENT state
            // Poll for a second?
            if (!timeout(1000))
            {
                sumDrogueDescentVel += stateStruct.vel_vert;
                droguePollCount += 1;
            }
            else
            {
                int16_t avgDescentRate = sumDrogueDescentVel / droguePollCount;
                if (avgDescentRate >= DROGUE_DESCENT_THRESHOLD)
                {
                    state_start = millis();
                    avionicsState = DROGUE_DESCENT;
                }
            }

            // ABORT Case
            // 10 second timeout
            if (timeout(DROGUE_DEPLOY_TIMEOUT))
            {
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
            if (altitude < (1500 * METER_CONVERSION))
            {
                state_start = millis();
                avionicsState = MAIN_DEPLOY;
                break;
            }

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
            if (!timeout(1000))
            {
                sumMainDescentVel += stateStruct.vel_vert;
                mainPollCount += 1;
            }
            else
            {
                int16_t avgDescentRate = sumMainDescentVel / mainPollCount;
                if (avgDescentRate >= MAIN_DESCENT_THRESHOLD)
                {
                    state_start = millis();
                    avionicsState = MAIN_DESCENT;
                }
            }

            // ABORT Case
            // 10 second timeout
            if (timeout(MAIN_DEPLOY_TIMEOUT))
            {
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
            abPct = 0;
            // stop all other processes
            // Start sending high-fidelity data backwards in time from abort trigger to launch from flash
            break;

        default:
            break;
        }

        // Pull data from accelerometer, rate gyroscope, magnetometer, and GPS
        readSensors();

        // Perform state estimation
        doStateEstimation();

        // Log data packer on Flash chip
        // logData();

        // Transmit data packet to ground station
        sendTelemetry();

        if (counter % 10 == 0)
        {
            debugPrint();
        }

        counter++;
        timer.reset();
    }
}
