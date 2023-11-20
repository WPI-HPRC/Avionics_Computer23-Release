#include <Arduino.h>
#include "lib/ServoDriver/ServoDriver.h"
#include "lib/MetroTimer/Metro.h"
#include "ControllerBoardLibraries/ControllerBoardConstants.h"
#include "ControllerBoardLibraries/Controller.h"
#include "ControllerBoardLibraries/StateEstimator.h"
#include "ControllerBoardLibraries/pitches.h"
#include "lib/Flash/Flash.h"
// #include "SensorBoardLibraries/SensorBoard.hpp"
#include "GroundStation/TelemetryBoard.h"
#include <EKF/StateEstimator.h>

#define AvionicsSim true // Simulates data through ground station as if it were a receiver

// Airbrake controller class
Controller controller;

// State estimation class
StateEstimator stateEstimator;

// Flash chip instantiation
FlashChip flash = FlashChip();
String structString = "";
String circBuf[FLASH_BUFFER_SIZE];
int circBufInd = 0;

// Telemetry Board class
TelemetryBoard telemBoard = TelemetryBoard();

// Sensor Board class
Sensorboard sensorboard;

// Airbrakes servo
ServoDriver airbrakeServo(SERVO_PIN); // Instantiate airbrake servo

// Main timer declarations
Metro timer = Metro(SECONDS / LOOP_FREQUENCY); // Hz converted to ms
int counter = 0;                                  // counts how many times the loop runs
unsigned long state_start;                        // must be set to millis() when entering a new state
uint32_t timestamp;                               // timestamp for telemetry packet

Metro prelaunchTimer = Metro(PRELAUNCH_INTERVAL); // 1 second timer to stay in STARTUP before moving to PRELAUNCH
Metro boostTimer = Metro(BOOST_MIN_LENGTH);       // 3 second timer to ensure BOOST state is locked before possibility of state change
Metro ab_start_timer = Metro(AB_START_DELAY);     // time interval to wait after motor burnout before deploying airbrakes
Metro ab_off_timer = Metro(AB_OFF_DELAY);         // time interval to keep airbrakes extended during coast
Metro coastTimer = Metro(COAST_MIN_LENGTH);       // 10 second timer to lock out apogee detect

// Variables for recording loop timing
long loopStartTime;
long loopTime;
long previousTime;

// Declarations for state transition detection buffers
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

// Pitches and durations for buzzer sounds
int startupPitches[] = {
    NOTE_C4, NOTE_G4, NOTE_C5, 0, NOTE_C4, NOTE_G4, NOTE_C5};

int startupDurations[] = {
    8, 8, 4, 2, 8, 8, 4};

int songPitches[] = {
    NOTE_D4, NOTE_F4, NOTE_D4, NOTE_D4, NOTE_G4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_A4, NOTE_D4, NOTE_D4, NOTE_AS4, NOTE_A4, NOTE_F4,
    NOTE_D4, NOTE_A4, NOTE_D5, NOTE_D4, NOTE_C4, NOTE_C4, NOTE_A3, NOTE_E4, NOTE_D4, 0, NOTE_D6, NOTE_D6};

int songDurations[] = {
    4, 6, 8, 16, 8, 8, 8, 4, 6, 8, 16, 8, 8, 8, 8, 8, 8, 16, 8, 16, 8, 8, 2, 16, 4, 4};

// Flag for boost timer
bool boostTimerElapsed = false;

// Flag for coast timer
bool coastFlag = false;

// Airbrake flags
bool abStartFlag = false;
bool abEndFlag = false;

// Variable for measured battery voltage
float vBatt = 0.0;

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
    STARTUP,        // Allows bootup of avionics
    PRELAUNCH,      // Holds initial data in a circular buffer until launch is detected, then dumps data to flash chip from the circular buffer and enters the BOOST state
    BOOST,          // Logs data to the flash chip, remains in the state for at least 3 seconds, then checks for motor burnout, then enters the COAST state
    COAST,          // Logs data to the flash chip, remains in the state for at least 10 seconds, then checks for apogee, then enters the DROGUE_DEPLOY state, runs airbrakes every 8th loop. If no change is triggered after 30 seconds, moves to DROGUE_DEPLOY, aborts if acceleration is over 10 Gs
    DROGUE_DEPLOY,  // Logs data to the flash chip, polls vertical velocity for 1 second, then sees if the average descent rate is below the threshold of 30 m/s, then moves to DROGUE_DESCENT if true. Aborts if been in state for longer than 10 seconds
    DROGUE_DESCENT, // Logs data to the flash chip, and checks if the altitude is below 1500 ft, if true - moves to the MAIN_DEPLOY state
    MAIN_DEPLOY,    // Logs data to the flash chip, polls vertical velocity for 1 second, then sees if the average descent rate is below the threshold of 7 m/s, then moves to MAIN_DESCENT if true. Aborts if been in state for longer than 10 seconds
    MAIN_DESCENT,   // Logs data to the flash chip, and checks for landing, if true - moves to POSTFLIGHT. Moves to POSTFLIGHT if it has been in the state for longer than 100 seconds
    POSTFLIGHT,     // Logs data to the flash chip
    ABORT,           // Logs data to the flash chip and fully retracts airbrakes
    SIMULATION
};

AvionicsState avionicsState = STARTUP;

// checks whether the state has timed out yet
bool timeout(uint32_t length)
{
    // check state length against current time - start time for the current state
    if (millis() - state_start >= length)
    {
        // true if the time in state has reached the value of 'length' in milliseconds
        return true;
    }
    else
    {
        return false;
    }
}

void readSensors();

// Assigns acceleration and gyroscope values to the average value polled over 1000 samples
void calibrateIMU()
{

    float ac_x_error_sum = 0;
    float ac_y_error_sum = 0;
    float ac_z_error_sum = 0;
    float gy_x_error_sum = 0;
    float gy_y_error_sum = 0;
    float gy_z_error_sum = 0;

    for (int i = 0; i < IMU_CALIBRATION_ITERS; i++)
    {

        readSensors();

        // Sum all readings
        ac_x_error_sum = ac_x_error_sum + sensorPacket.ac_x;
        ac_y_error_sum = ac_y_error_sum + sensorPacket.ac_y;
        // ac_z_error_sum = ac_z_error_sum + sensorPacket.ac_z;
        gy_x_error_sum = gy_x_error_sum + sensorPacket.gy_x;
        gy_y_error_sum = gy_y_error_sum + sensorPacket.gy_y;
        gy_z_error_sum = gy_z_error_sum + sensorPacket.gy_z;
    }

    // Divide the sum by 1000 to get the error value
    ac_x_error = ac_x_error_sum / IMU_CALIBRATION_ITERS;
    ac_y_error = ac_y_error_sum / IMU_CALIBRATION_ITERS;
    // ac_z_error = ac_z_error_sum / IMU_CALIBRATION_ITERS;
    gy_x_error = gy_x_error_sum / IMU_CALIBRATION_ITERS;
    gy_y_error = gy_y_error_sum / IMU_CALIBRATION_ITERS;
    gy_z_error = gy_z_error_sum / IMU_CALIBRATION_ITERS;

    Serial.println("IMU Calibration Complete");
}

float pressureToAltitude(float pressure_mBar);

// Defines the above ground level altitude to be the average altitude over 2000 samples
void calibrateAltitudeAGL()
{

    float altitude_error_sum = 0;

    for (int i = 0; i < AGL_CALIBRATION_ITERS; i++)
    {

        readSensors();
        altitude = pressureToAltitude(sensorPacket.Pressure);

        // Sum all readings
        altitude_error_sum = altitude_error_sum + altitude;
    }

    // Divide the sum to get the error value
    altitude_AGL = altitude_error_sum / AGL_CALIBRATION_ITERS;

    Serial.print("altitude_AGL: ");
    Serial.println(altitude_AGL);
}

// uses updated accel value to determine if the rocket has launched
// stores 10 most recent accels and determines if average is greater than the threshold
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
        // Serial.println("Launch detected!");
        return true;
    }
    return false;
}

// detects if motor burnout has occurred (aka negative accel)
// tracks 10 most recent accels and determines if average is negative
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

    Serial.println(sum);

    transitionBufIndAcc = (transitionBufIndAcc + 1) % 10;
    // compare running average value to defined threshold
    if (sum < 0)
    {
        for (int j = 0; j < 10; j++)
        {
            transitionBufAcc[j] = 0;
        }
        transitionBufIndAcc = 0;
        // Serial.println("Motor burnout detected!");
        return true;
    }
    return false;
}

// detects if the rocket has reached apogee
// tracks the 10 most recent vertical velocities and determines if the average is negative
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
        // Serial.println("Apogee detected!");
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
        // Serial.println("Landing detected!");
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
    telemPacket.timestamp = millis() - loopStartTime;

    // State
    telemPacket.state = (uint8_t)avionicsState;

    // Battery voltage
    telemPacket.vBatt = (uint8_t)(vBatt * 20); // TODO what is 20?

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

}

// Read sensors and construct telemPacket
void readSensors()
{
    // Construct sensorPacket struct with raw data from inertial sensors + barometer
    sensorboard.readInertialSensors();
    memcpy(&sensorPacket, &sensorboard.Inertial_Baro_frame, sizeof(sensorboard.Inertial_Baro_frame));
    sensorPacket.ac_x -= ac_x_error;
    sensorPacket.ac_y -= ac_y_error;
    sensorPacket.ac_z -= ac_z_error;
    sensorPacket.gy_x -= gy_x_error;
    sensorPacket.gy_y -= gy_y_error;
    sensorPacket.gy_z -= gy_z_error;

    // Calculate total acceleration
    ac_total = sqrt((sensorPacket.ac_x * sensorPacket.ac_x + sensorPacket.ac_y * sensorPacket.ac_y + sensorPacket.ac_z * sensorPacket.ac_z));

    // Compute altitude from pressure
    altitude = pressureToAltitude(sensorPacket.Pressure) - altitude_AGL;

    // Read battery voltage
    vBatt = ((float)analogRead(VBATT_PIN) / (ADC_RESOLUTION / ADC_FULL_SCALE_RANGE)) * VOLTAGE_DIVIDER_RATIO;

    // Construct telemetry data packet
    constructTelemPacket();
}

// adds a new telemPacket to the circular buffer
void buildCircBuf()
{
    structString = String(telemPacket.timestamp) + "," + String(telemPacket.state) + "," + String(telemPacket.altitude) + "," + String(telemPacket.temperature) + "," + String(telemPacket.abPct) + "," + String(telemPacket.ac_x) + "," + String(telemPacket.ac_y) + "," + String(telemPacket.ac_z) + "," + String(telemPacket.gy_x) + "," + String(telemPacket.gy_y) + "," + String(telemPacket.gy_z) + "," + String(telemPacket.vel_vert) + "," + String(telemPacket.vel_lat) + "," + String(telemPacket.vel_total) + "," + String(telemPacket.vBatt);
    circBuf[circBufInd] = structString;  // pass in the string to the circular buffer
    circBufInd = (circBufInd + 1) % 200; // increment the index and wrap around if necessary
}

// writes circular buffer to flash chip
void writeCircBuf()
{
    for (int i = 0; i < 200; i++)
    {
        flash.writeStruct(circBuf[i]);
    }
}

// Write data (telemPacket) to flash chip
void logData()
{
    structString = String(telemPacket.timestamp) + "," + String(telemPacket.state) + "," + String(telemPacket.altitude) + "," + String(telemPacket.temperature) + "," + String(telemPacket.abPct) + "," + String(telemPacket.ac_x) + "," + String(telemPacket.ac_y) + "," + String(telemPacket.ac_z) + "," + String(telemPacket.gy_x) + "," + String(telemPacket.gy_y) + "," + String(telemPacket.gy_z) + "," + String(telemPacket.vel_vert) + "," + String(telemPacket.vel_lat) + "," + String(telemPacket.vel_total) + "," + String(telemPacket.vBatt);
    flash.writeStruct(structString);
}

// Transmit telemetry packet to ground station
void sendTelemetry()
{
    telemBoard.onLoop(telemPacket);
}

// Compute airbrake actuation percent from controller class.
//  Inputs: vertical velocity, lateral velocity, altitude
//  Output: airbrake actuation level as a percent from 0 to 100
void doAirbrakeControls()
{
    abPct = controller.calcAbPct(altitude, stateStruct.vel_lat, stateStruct.vel_vert);
    airbrakeServo.setPosition(abPct);
}

// Print telemPacket to Serial monitor for debugging purposes
void debugPrint()
{
    Serial.println("");
    Serial.print("Timestamp: ");
    Serial.print(telemPacket.timestamp);
    Serial.println(" ms");
    Serial.print("State: ");
    Serial.println(telemPacket.state);
    Serial.print("Battery voltage: ");
    Serial.print(telemPacket.vBatt / 20.0);
    Serial.println(" V");
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
    Serial.print((float)telemPacket.ac_x / 100.0);
    Serial.println(" g");
    Serial.print("Accel-Y: ");
    Serial.print((float)telemPacket.ac_y / 100.0);
    Serial.println(" g");
    Serial.print("Accel-Z: ");
    Serial.print((float)telemPacket.ac_z / 100.0);
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
    Serial.print(stateStruct.vel_vert);
    Serial.println(" m/s");
    Serial.print("Lateral velocity: ");
    Serial.print(stateStruct.vel_lat);
    Serial.println(" m/s");
    Serial.print("Total velocity: ");
    Serial.print(stateStruct.vel_total);
    Serial.println(" m/s");
    Serial.println("");
}

// Turn on the LED of the given color
void LEDon(String color)
{
    if (color == "RED")
    {
        digitalWrite(RED_LED, HIGH);
    }
    else if (color == "YELLOW")
    {
        digitalWrite(YELLOW_LED, HIGH);
    }
    else if (color == "BLUE")
    {
        digitalWrite(BLUE_LED, HIGH);
    }
}

// Turn off the LED of the given color
void LEDoff(String color)
{
    if (color == "RED")
    {
        digitalWrite(RED_LED, LOW);
    }
    else if (color == "YELLOW")
    {
        digitalWrite(YELLOW_LED, LOW);
    }
    else if (color == "BLUE")
    {
        digitalWrite(BLUE_LED, LOW);
    }
}

// Turn on all LEDs
void LEDsOn()
{
    LEDon("RED");
    LEDon("YELLOW");
    LEDon("BLUE");
}

// Turn off all LEDs
void LEDsOff()
{
    LEDoff("RED");
    LEDoff("YELLOW");
    LEDoff("BLUE");
}

// Initialize LED pins
void initLEDs()
{
    pinMode(RED_LED, OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
}

// Blink the yellow LED as a heartbeat indicator in the main loop
void blinkLED()
{
    if (counter % 40 == 0)
    {
        LEDon("YELLOW");
    }
    else if (counter % 40 == 2)
    {
        LEDoff("YELLOW");
    }
}

// Play a few beeps to indicate power on.
void startupBeeps()
{
    for (int thisNote = 0; thisNote < 7; thisNote++)
    {
        int noteDuration = 1000 / startupDurations[thisNote];
        tone(BUZZER_PIN, startupPitches[thisNote], noteDuration);
        int pauseBetweenNotes = noteDuration * 1.30;
        delay(pauseBetweenNotes);
        noTone(BUZZER_PIN);
    }
}

// Play a song to indicate succesful setup and entering main loop.
void mainLoopBeeps()
{
    for (int thisNote = 0; thisNote < 26; thisNote++)
    {
        int noteDuration = 1400 / songDurations[thisNote];
        tone(BUZZER_PIN, songPitches[thisNote], noteDuration);
        int pauseBetweenNotes = noteDuration * 1.30;
        delay(pauseBetweenNotes);
        noTone(BUZZER_PIN);
    }
}

// Extend airbrake fins to maximum extension for 1 second, then retract.
void testAirbrakes()
{
    // Serial.println("Testing airbrakes!");
    airbrakeServo.setPosition(100);
    delay(2000);
    airbrakeServo.setPosition(0);
}

// Print to the Serial monitor the measured duration of one loop of the flight computer.
void printLoopTime()
{
    long currentTime = millis();
    loopTime = currentTime - previousTime;
    previousTime = currentTime;
    Serial.print("Loop time: ");
    Serial.print(loopTime);
    Serial.println(" ms");
}

// Change state to coast and reset relevant timers
void transitionToCoast() {
    state_start = millis();
    avionicsState = COAST;
    coastTimer.reset();
    ab_start_timer.reset();
    ab_off_timer.reset();
}

// Change state to drogue deploy and reset relevant timers
void transitionToDrogueDeploy() {
    abPct = 0;
    airbrakeServo.setPosition(0);
    state_start = millis();
    avionicsState = DROGUE_DEPLOY;
}

// Setup Kalman Filter an Obtain Initial Estimate
QuatStateEstimator * estimator;
constexpr static int initialLoopIters = 100;
// Initialize current state variable with zeros
BLA::Matrix<4> currentState = {0,0,0,0};

BLA::Matrix<4> obtainInitialEstimate() {
    float accelXSum = 0;
    float accelYSum = 0;
    float accelZSum = 0;

    for(int i=0; i < initialLoopIters; i++) {
        readSensors();
        accelXSum += sensorPacket.ac_x * 9.81;
        accelYSum += sensorPacket.ac_y * 9.81;
        accelZSum += sensorPacket.ac_z * 9.81;
    };

    // Get average of sensors
    float accelXAvg = accelXSum / initialLoopIters;
    float accelYAvg = accelYSum / initialLoopIters;
    float accelZAvg = accelZSum / initialLoopIters; 

    // Get euler angles
    float roll_0 = atan(accelYAvg / accelZAvg); // [rad]
    float pitch_0 = atan(accelXAvg / G); // [rad]
    float yaw_0 = 0; // [rad]

    // Get quaternion
    float cr = cos(roll_0 * 0.5);
    float sr = sin(roll_0 * 0.5);
    float cp = cos(pitch_0 * 0.5);
    float sp = sin(pitch_0 * 0.5);
    float cy = cos(yaw_0 * 0.5);
    float sy = sin(yaw_0 * 0.5);

    BLA::Matrix<4> x_update = {
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    };

    return x_update;
};

// Built-in Arduino setup function. Performs initialization tasks on startup.
void setup()
{
    // Initialize airbrake servo and set to fully retracted position
    //      IMPORTANT: This needs to be the first thing in setup()
    airbrakeServo.init();

    // Communications setup
    Serial.begin(57600);

    // Setup push button and buzzer
    pinMode(BUTTON_PIN, INPUT);
    pinMode(VBATT_PIN, INPUT);

    // LEDs for debugging
    initLEDs();
    LEDsOn();

    // Play tones on startup
    startupBeeps();

    // Perform airbrake extension test if button pressed
    if (digitalRead(BUTTON_PIN))
    {
        testAirbrakes();
    }

    // Telemetry initialization
    telemBoard.setState(SIM);
    telemBoard.init();

    // Flash memory initialization
    flash.init();
    int startingAddress = 0;
    if (telemBoard.getState() == TX)
    {
        startingAddress = flash.rememberAddress();
    }
    // Serial.print("Flash Starting: ");
    // Serial.println(startingAddress);

    // Sensor initialization
    Wire.begin();
    // Wire.setClock(400000);
    SPI.begin();

    Serial.println("Initializing sensor board...");
    if (sensorboard.setup())
    {
        Serial.println("Sensor setup success!");
    }
    else
    {
        Serial.println("Sensor setup failed.");
    }

    // Run calibration routine if the board is a transmitter
    if (telemBoard.getState() == TX)
    {
        // IMU calibration
        calibrateIMU();

        // Altitude AGL compensation
        calibrateAltitudeAGL();

        // Set initial conditions for controller class
        controller.setInitPressureTemp(sensorPacket.Pressure, sensorPacket.Temperature);

        // Play funny song before loop
        mainLoopBeeps();

        // Turn off LEDs to indicate initialization complete
        LEDsOff();
    } else if(telemBoard.getState() == SIM) {

        calibrateIMU();
        calibrateAltitudeAGL();
    }

    BLA::Matrix<4> x_0 = obtainInitialEstimate();

    Serial.println("----- Initial State -----");
    Serial.println("W: " + String(x_0(0)));
    Serial.println("I: " + String(x_0(1)));
    Serial.println("J: " + String(x_0(2)));
    Serial.println("K: " + String(x_0(3)));

    estimator = new QuatStateEstimator(x_0, 0.025);

    // Reset timer before entering loop
    timer.reset();
    previousTime = millis();

    // Set system time to zero
    loopStartTime = millis() + STARTUP_DELAY; // 25 millis for setup
}

// Built-in Arduino loop function. Executes main control loop at a specified frequency.
void loop()
{
    // controls how frequently this loop runs, defined by LOOP_FREQUENCY in the constants file
    if (timer.check() == 1)
    {

        // printLoopTime();

        switch (avionicsState)
        {

        case STARTUP:
            // STARTUP state must be active for at least 1 second
            if (prelaunchTimer.check() == 1)
            {
                avionicsState = PRELAUNCH;
                state_start = millis();
            }
            if(AvionicsSim) {
                avionicsState = SIMULATION;
                state_start = millis();
            }
            break;

        case PRELAUNCH:
            // detect acceleration of 3G's
            buildCircBuf();
            if (launchDetect())
            {
                writeCircBuf();
                avionicsState = BOOST;
                boostTimer.reset();
                state_start = millis();
            }
            break;

        case BOOST:

            logData();

            // Stay in this state for at least 3 seconds to prevent airbrake activation
            if (boostTimer.check() == 1)
            {
                boostTimerElapsed = true;
            }

            if (boostTimerElapsed)
            {
                if (motorBurnoutDetect())
                {
                    transitionToCoast();
                    break;
                }
            }

            // Wait 8 seconds before moving into COAST state by default
            if (timeout(BOOST_TIMEOUT))
            {
                Serial.println("Boost phase timeout triggered!");
                transitionToCoast();
                break;
            }

            break;

        case COAST:

            logData();

            if (counter % 8 == 0)
            {
                // doAirbrakeControls();
            }

            if (coastTimer.check() == 1)
            {
                coastFlag = true;
            }

            if (ab_start_timer.check() == 1)
            {
                abStartFlag = true;
            }

            if (ab_off_timer.check() == 1) 
            {
                abEndFlag = true;
            }

            if (abStartFlag && !abEndFlag)
            {
                abPct = CONST_EXTENSION;
                airbrakeServo.setPosition(CONST_EXTENSION);
            } else {
                abPct = 0;
                airbrakeServo.setPosition(0);
            }

            if (coastFlag)
            {
                if (apogeeDetect())
                {
                    Serial.println("Apogee detected!");
                    transitionToDrogueDeploy();
                }
            }

            // Wait 30 seconds before moving into DROGUE_DEPLOY state if all else fails
            if (timeout(COAST_TIMEOUT))
            {
                Serial.println("Coast phase timeout triggered!");
                transitionToDrogueDeploy();
                break;
            }

            // Acceleration is greater than 10g's
            // 10-15 second timeout on 10g check to ensure ejection load doesn't accidentally trigger ABORT
            if (ac_total > 10)
            {
                if (timeout(13000)) // currently 13 seconds
                {
                    // Serial.println("Abort triggered by acceleration greate than 10g's");
                    state_start = millis();
                    avionicsState = ABORT;
                    break;
                }
            }
            break;

        case DROGUE_DEPLOY:
            logData();
            // 1 second timeout to average vertical velocity
            if (!timeout(2000))
            {
                sumDrogueDescentVel += stateStruct.vel_vert;
                droguePollCount += 1;
            }
            else
            {
                int16_t avgDescentRate = sumDrogueDescentVel / droguePollCount;
                if (abs(avgDescentRate) <= DROGUE_DESCENT_THRESHOLD)
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

        case DROGUE_DESCENT:
            logData();
            // detect altitude drop below 1500ft
            if (altitude < (1500 * METER_CONVERSION))
            {
                state_start = millis();
                avionicsState = MAIN_DEPLOY;
                break;
            }
            break;

        case MAIN_DEPLOY:
            logData();
            // 1 second timeout to average vertical velocity
            if (!timeout(5000))
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

        case MAIN_DESCENT:
            logData();
            // transmit data during this phase

            // altitude below 20 meters
            if (landingDetect())
            {
                state_start = millis();
                avionicsState = POSTFLIGHT;
            }

            // timeout after 100 seconds if other checks fail
            if (timeout(MAIN_DESCENT_TIMEOUT))
            {
                // Serial.println("Main Descent phase timeout triggered!");
                avionicsState = POSTFLIGHT;
                break;
            }
            break;

        case POSTFLIGHT:
            logData();
            // datalog slow
            break;

        case ABORT:
            logData();
            // jump to here if anything goes terribly wrong (but obv it won't)
            // retract airbrakes
            abPct = 0;
            airbrakeServo.setPosition(0);
            break;
        
        case SIMULATION:
            break;

        default:
            break;
        }

        // Blink the yellow LED as a heartbeat indicator that the loop is running
        blinkLED();

        // Pull data from accelerometer, rate gyroscope, magnetometer, and GPS
        readSensors();

        // Perform state estimation
        currentState = estimator->onLoop(sensorPacket);

        if(counter % 1 == 0) {
            // Serial.println("----- CURRENT STATE -----");
            // Serial.println("W: " + String(currentState(0)));
            // Serial.println("I: " + String(currentState(1)));
            // Serial.println("J: " + String(currentState(2)));
            // Serial.println("K: " + String(currentState(3)));
        };

        // Transmit data packet to ground station at 10 Hz
        if (counter % 1 == 0)
        {
            // sendTelemetry();
        }

        // Print telemPacket to Serial monitor for debugging
        if (counter % 10 == 0)
        {
            // debugPrint();
        }

        counter++;
    }
}