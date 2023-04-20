#include <Arduino.h>
#include "lib/ServoDriver/ServoDriver.h"
#include "lib/MetroTimer/Metro.h"
#include "ControllerBoardLibraries/ControllerBoardConstants.h"
#include "ControllerBoardLibraries/Controller.h"
#include "ControllerBoardLibraries/StateEstimator.h"
#include "lib/Flash/Flash.h"
#include "SensorBoardLibraries/SensorBoard.hpp"
#include "GroundStation/TelemetryBoard.h"

long loopTime;
long previousTime;

// Airbrake controller class
Controller controller;

// State estimation class
StateEstimator stateEstimator;

// Flash chip instantiation
FlashChip flash = FlashChip();
String structString = "";

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
uint32_t timestamp;                               // timestamp for telemetry packet

Metro prelaunchTimer = Metro(PRELAUNCH_INTERVAL); // 1 second timer to stay in STARTUP before moving to PRELAUNCH
Metro boostTimer = Metro(BOOST_MIN_LENGTH);       // 3 second timer to ensure BOOST state is locked before possibility of state change
Metro coastTimer = Metro(COAST_MIN_LENGTH);       // 10 second timer to ensure COAST state is locked before possibility of state change

// Declarations for state transition detection buffer
// Z Acceleration buffer
float transitionBufAcc[10];
uint8_t transitionBufIndAcc = 0;

// Vertical velocity buffer
int16_t transitionBufVelVert[10];
uint8_t transitionBufIndVelVert = 0;

// Altitude buffer
int16_t transitionBufAlt[10];
uint8_t transitionBufIndAlt = 0;
int16_t altitudePreviousAvg;

// Declaration for sensor data struct, measured values
SensorFrame sensorPacket;

// Declaration for GPS data struct
GPSFrame gpsPacket;

// Declaration for telemetry packet struct, calculated values
TelemetryPacket telemPacket;

// Declaration for state estimate struct
StateStruct stateStruct;

// Variables for IMU biases
float ac_x_error;
float ac_y_error;
float ac_z_error;
float gy_x_error;
float gy_y_error;
float gy_z_error;

// Ground level altitude compensation
float altitude_AGL;

// Variable declarations for filtered state data
float altitude;
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

AvionicsState avionicsState = COAST;

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

void readSensors();

void calibrateIMU()
{

    float ac_x_error_sum = 0;
    float ac_y_error_sum = 0;
    float ac_z_error_sum = 0;
    float gy_x_error_sum = 0;
    float gy_y_error_sum = 0;
    float gy_z_error_sum = 0;

    int c = 0;
    while (c < 1000)
    {

        sensorboard.readInertialSensors();
        memcpy(&sensorPacket, &sensorboard.Inertial_Baro_frame, sizeof(sensorboard.Inertial_Baro_frame));

        // Sum all readings
        ac_x_error_sum = ac_x_error_sum + sensorPacket.ac_x;
        ac_y_error_sum = ac_y_error_sum + sensorPacket.ac_y;
        ac_z_error_sum = ac_z_error_sum + sensorPacket.ac_z;
        gy_x_error_sum = gy_x_error_sum + sensorPacket.gy_x;
        gy_y_error_sum = gy_y_error_sum + sensorPacket.gy_y;
        gy_z_error_sum = gy_z_error_sum + sensorPacket.gy_z;

        c++;
    }

    // Divide the sum by 12000 to get the error value
    ac_x_error = ac_x_error_sum / c;
    ac_y_error = ac_y_error_sum / c;
    ac_z_error = ac_z_error_sum / c - 1.0;
    gy_x_error = gy_x_error_sum / c;
    gy_y_error = gy_y_error_sum / c;
    gy_z_error = gy_z_error_sum / c;

    Serial.print("ac_x_error: ");
    Serial.println(ac_x_error);
    Serial.print("ac_y_error: ");
    Serial.println(ac_y_error);
    Serial.print("ac_z_error: ");
    Serial.println(ac_z_error);
    Serial.print("gy_x_error: ");
    Serial.println(gy_x_error);
    Serial.print("gy_y_error: ");
    Serial.println(gy_y_error);
    Serial.print("gy_z_error: ");
    Serial.println(gy_z_error);
}

float pressureToAltitude(float pressure_mBar);

void calibrateAltitudeAGL()
{

    float altitude_error_sum = 0;

    int c = 0;
    while (c < 2000)
    {

        readSensors();
        altitude = pressureToAltitude(sensorPacket.Pressure);

        // Sum all readings
        altitude_error_sum = altitude_error_sum + altitude;

        c++;
    }

    // Divide the sum -to get the error value
    altitude_AGL = altitude_error_sum / c;

    Serial.print("altitude_AGL: ");
    Serial.println(altitude_AGL);
}

// uses updated accel value to determine if the rocket has launched
// stores 10 most recent values and computes current avg
boolean launchDetect()
{
    // accel value gets updated in sensor reading fcn
    // add to cyclic buffer
    transitionBufAcc[transitionBufIndAcc] = sensorPacket.ac_z;
    // take running average value
    float sum = 0.0;
    for (int i = 0; i < 10; i++)
    {
        sum += transitionBufAcc[i];
    }
    sum = sum / 10.0;
    transitionBufIndAcc = (transitionBufIndAcc + 1) % 10;
    // compare running average value to defined threshold
    if (sum > ACCEL_THRESHOLD)
    {
        for (int j = 0; j < 10; j++)
        {
            transitionBufAcc[j] = 0;
        }
        transitionBufIndAcc = 0;
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
    transitionBufAcc[transitionBufIndAcc] = sensorPacket.ac_z;
    // take running average value
    float sum = 0.0;
    for (int i = 0; i < 10; i++)
    {
        sum += transitionBufAcc[i];
    }
    sum = sum / 10.0;

    transitionBufIndAcc = (transitionBufIndAcc + 1) % 10;
    // compare running average value to defined threshold
    if (sum < 0)
    {
        for (int j = 0; j < 10; j++)
        {
            transitionBufAcc[j] = 0;
        }
        transitionBufIndAcc = 0;
        Serial.println("Motor burnout detected!");
        return true;
    }
    return false;
}

boolean apogeeDetect()
{
    // Velocity value gets updated in sensor reading fcn
    // add to cyclic buffer
    transitionBufVelVert[transitionBufIndVelVert] = stateStruct.vel_vert;
    // take running average value
    float sum = 0.0;
    for (int i = 0; i < 10; i++)
    {
        sum += transitionBufVelVert[i];
    }
    sum = sum / 10.0;

    transitionBufIndVelVert = (transitionBufIndVelVert + 1) % 10;
    // if average vertical velocity is negative, passed apogee
    if (sum < 0)
    {
        for (int j = 0; j < 10; j++)
        {
            transitionBufVelVert[j] = 0;
        }
        transitionBufIndVelVert = 0;
        Serial.println("Apogee detected!");
        return true;
    }
    return false;
}

// detect landing by registering when the altitude doesn't change much
boolean landingDetect()
{
    // ALtitude value gets updated in sensor reading fcn
    // add to cyclic buffer
    transitionBufAlt[transitionBufIndAlt] = altitude;

    // take running average value
    float avg_sum = 0.0;
    for (int i = 0; i < 10; i++)
    {
        avg_sum += transitionBufAcc[i];
    }
    float avg = avg_sum / 10.0;

    // calculate mean difference from average
    float avg_std_sum = 0.0;
    for (int i = 0; i < 10; i++)
    {
        avg_std_sum += abs(transitionBufAcc[i] - avg);
    }
    float avg_std = avg_std_sum / 10.0;

    // if altitude is nearly constant and below landing threshold
    if (avg_std < 5 && altitude <= LAND_THRESHOLD)
    {
        for (int j = 0; j < 10; j++)
        {
            transitionBufAlt[j] = 0;
        }
        transitionBufIndAlt = 0;
        Serial.println("Landing detected!");
        return true;
    }
    transitionBufIndAlt = (transitionBufIndAlt + 1) % 10;

    return false;
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
    timestamp = counter * (CONVERSION / LOOP_FREQUENCY);
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
    telemPacket.ac_x = (int16_t)(sensorPacket.ac_x * 100.0);
    telemPacket.ac_y = (int16_t)(sensorPacket.ac_y * 100.0);
    telemPacket.ac_z = (int16_t)(sensorPacket.ac_z * 100.0);

    // Angular rate (XYZ) [deg/s] - Scaled by 10x for transmission
    telemPacket.gy_x = (int16_t)(sensorPacket.gy_x * 10.0);
    telemPacket.gy_y = (int16_t)(sensorPacket.gy_y * 10.0);
    telemPacket.gy_z = (int16_t)(sensorPacket.gy_z * 10.0);

    // Velocity (vertical, lateral, total) [m/s]
    telemPacket.vel_vert = (int16_t)(stateStruct.vel_vert * 100.0);
    telemPacket.vel_lat = (int16_t)(stateStruct.vel_lat * 100.0);
    telemPacket.vel_total = (int16_t)(stateStruct.vel_total * 100.0);

    // TODO: Add GPS to telemPacket
}

// Read sensors and construct telemPacket
void readSensors()
{
    // Construct sensorPacket struct with raw data from inertial sensors + barometer
    sensorboard.readInertialSensors();
    memcpy(&sensorPacket, &sensorboard.Inertial_Baro_frame, sizeof(sensorboard.Inertial_Baro_frame));

    sensorPacket.ac_x -= ac_x_error;
    sensorPacket.ac_y -= ac_y_error;
    sensorPacket.ac_z -= (1.0 + ac_z_error);
    sensorPacket.gy_x -= gy_x_error;
    sensorPacket.gy_y -= gy_y_error;
    sensorPacket.gy_z -= gy_z_error;

    // Calculate total acceleration
    ac_total = sqrt((sensorPacket.ac_x * sensorPacket.ac_x + sensorPacket.ac_y * sensorPacket.ac_y + sensorPacket.ac_z * sensorPacket.ac_z));

    // Compute altitude from pressure
    altitude = pressureToAltitude(sensorPacket.Pressure) - altitude_AGL;

    // Construct gpsPacket struct with data from GPS
    // TODO: Get GPS data from sensor board class

    // Construct telemetry data packet
    constructTelemPacket();
}

// Write data (telemPacket) to flash chip
void logData()
{
    structString = String(telemPacket.timestamp) + "," + String(telemPacket.state) + "," + String(telemPacket.altitude) + "," + String(telemPacket.temperature) + "," + String(telemPacket.abPct) + "," + String(telemPacket.ac_x) + "," + String(telemPacket.ac_y) + "," + String(telemPacket.ac_z) + "," + String(telemPacket.gy_x) + "," + String(telemPacket.gy_y) + "," + String(telemPacket.gy_z) + "," + String(telemPacket.vel_vert) + "," + String(telemPacket.vel_lat) + "," + String(telemPacket.vel_total);

    flash.writeStruct(structString);
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
    stateStruct = stateEstimator.getState(altitude, sensorPacket.ac_x, sensorPacket.ac_y, sensorPacket.ac_z);
}

// Compute airbrake actuation percent from controller class.
//  Inputs: vertical velocity, lateral velocity, altitude
//  Output: airbrake actuation level as a percent from 0 to 100
void doAirbrakeControls()
{
    abPct = controller.calcAbPct(altitude, stateStruct.vel_lat, stateStruct.vel_vert);
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
    Serial.println(" g");
    Serial.print("Accel-Y: ");
    Serial.print(telemPacket.ac_y / 100.0);
    Serial.println(" g");
    Serial.print("Accel-Z: ");
    Serial.print(telemPacket.ac_z / 100.0);
    Serial.println(" g");
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
    Serial.print(telemPacket.vel_vert / 100.0);
    Serial.println(" m/s");
    Serial.print("Lateral velocity: ");
    Serial.print(telemPacket.vel_lat / 100.0);
    Serial.println(" m/s");
    Serial.print("Total velocity: ");
    Serial.print(telemPacket.vel_total / 100.0);
    Serial.println(" m/s");
    Serial.println("");
}

void LEDon()
{
    digitalWrite(LED_PIN, HIGH);
}

void LEDoff()
{
    digitalWrite(LED_PIN, LOW);
}

void initLED()
{
    pinMode(LED_PIN, OUTPUT);
    LEDon();
}

void blinkLED()
{
    if (counter % 100 == 0)
    {
        LEDon();
    }
    else if (counter % 100 == 10)
    {
        LEDoff();
    }
}

// Built-in Arduino setup function. Performs initilization tasks on startup.
void setup()
{
    // Communications setup
    Serial.begin(57600);

    // LED for debugging
    initLED();

    // Initialize airbrake servo and set to fully retracted position
    airbrakeServo.init();
    airbrakeServo.setPosition(0);

    // Telemetry initialization
    telemBoard.setState(TX);
    telemBoard.init();

    // Flash memory initialization
    // flash.init();

    // Sensor initialization
    Wire.begin();
    Wire.setClock(400000);
    SPI.begin();

    if (sensorboard.setup())
    {
        Serial.println("Sensor setup sucess!");
    }
    else
    {
        Serial.println("Sensor setup failed.");
    }

    // IMU calibration
    calibrateIMU();

    // Altitude AGL compensation
    calibrateAltitudeAGL();

    // Set initial conditions for controller class
    controller.setInitPressureTemp(sensorPacket.Pressure, sensorPacket.Temperature);

    LEDoff();
}

void printLoopTime()
{
    long currentTime = millis();
    loopTime = currentTime - previousTime;
    previousTime = currentTime;
    Serial.print("Loop time: ");
    Serial.print(loopTime);
    Serial.println(" ms");
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
                    coastTimer.reset();
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
            // if ((ac_total > 10) || (stateStruct.vel_lat / stateStruct.vel_vert > PITCH_FRACTION))
            // {
            //     state_start = millis();
            //     avionicsState = ABORT;
            //     break;
            // }

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

            // Lock out apogee detection for 10 seconds to prevent false apogee detection
            if (coastTimer.check() == 1)
            {
                Serial.println("Apogee detection enabled!");
                if (apogeeDetect())
                {
                    abPct = 0;
                    airbrakeServo.setPosition(0); // Retract airbrakes fully upon apogee detecetion
                    // TODO: Add some delay on a timer to ensure airbrakes get fully retracted
                    state_start = millis();
                    avionicsState = DROGUE_DEPLOY;
                }
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
            // if (stateStruct.vel_lat / stateStruct.vel_vert > PITCH_FRACTION)
            // {
            //     state_start = millis();
            //     avionicsState = ABORT;
            //     break;
            // }

            // Acceleration is greater than 10g's
            // 10-15 second timeout on 10g check to ensure ejection load doesn't accidentally trigger ABORT
            if (ac_total > 10)
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
            if (!timeout(1000))
            {
                sumDrogueDescentVel += stateStruct.vel_vert;
                droguePollCount += 1;
            }
            else
            {
                int16_t avgDescentRate = sumDrogueDescentVel / droguePollCount;
                if (avgDescentRate <= DROGUE_DESCENT_THRESHOLD)
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
                if (avgDescentRate <= MAIN_DESCENT_THRESHOLD)
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
            airbrakeServo.setPosition(0);
            // stop all other processes
            // Start sending high-fidelity data backwards in time from abort trigger to launch from flash
            break;

        default:
            break;
        }

        blinkLED();

        // Pull data from accelerometer, rate gyroscope, magnetometer, and GPS
        readSensors();

        // Perform state estimation
        doStateEstimation();

        // Log data packer on Flash chip
        logData();

        if (counter % 10 == 0)
        {
            // Transmit data packet to ground station
            sendTelemetry();

            // Print telemPacket to Serial monitor for debugging
            // debugPrint();
        }

        printLoopTime();

        counter++;
        timer.reset();
    }
}
