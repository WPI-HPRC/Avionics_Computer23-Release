#include <Arduino.h>
#include "Libraries/MetroTimer/Metro.h"
#include "ControllerBoardConstants.h"
#include "Libraries/ACAN2517FD/ACAN2517FD.h"

// CAN Setup
static const byte MCP2518FD_CS = 10;
static const byte MCP2518FD_INT = 2;

ACAN2517FD can(MCP2518FD_CS, SPI, MCP2518FD_INT);

// Main timer declarations
Metro timer = Metro(CONVERSION / LOOP_FREQUENCY); // Hz converted to ms
int counter = 0;                                  // counts how many times the loop runs
unsigned long state_start;

Metro prelaunchTimer = Metro(PRELAUNCH_INTERVAL);   // 1 second timer to stay in STARTUP before moving to PRELAUNCH
Metro burnoutTimer = Metro(MOTOR_BURNOUT_INTERVAL); // 1 second timer after motor burnout detected
Metro boostTimer = Metro(BOOST_MIN_LENGTH);         // 3 second timer to ensure BOOST state is locked before possibility of state change

// must be set to millis() when entering a new state
unsigned long state_start;

// Declarations for state transition detection buffer
int16_t transitionBuf[10];
uint8_t transitionBufInd = 0;

// Declaration of data packet array
uint8_t DataPacket[24] = {0};

// Variable declarations for raw sensor data
uint32_t timestamp;
float pressure;
int8_t temperature;
uint8_t vBatt;
int16_t acX;
int16_t acY;
int16_t acZ;
int16_t gyX;
int16_t gyY;
int16_t gyZ;
int16_t magX;
int16_t magY;
int16_t magZ;

// Variable declarations for filtered state data
int16_t posX;
int16_t posY;
int16_t posZ;
int16_t velX;
int16_t velY;
int16_t velZ;
uint8_t abPct;

enum AvionicsState
{
    STARTUP,
    PRELAUNCH,
    BOOST,
    // COAST_CONTINGENCY,
    COAST,
    DROGUE_DEPLOY,
    // DROGUE_CONTINGENCY,
    DROGUE_DESCENT,
    // MAIN_CONTINGENCY,
    MAIN_DEPLOY,
    MAIN_DESCENT,
    POSTFLIGHT,
    ABORT
};

AvionicsState avionicsState = STARTUP;

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

// uses updated accel value to determine if the rocket has launched
// stores 10 most recent values and computes current avg
boolean launchDetect()
{
    // accel value gets updated in sensor reading fcn
    // add to cyclic buffer
    transitionBuf[transitionBufInd] = acZ;
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
    transitionBuf[transitionBufInd] = acZ;
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
    transitionBuf[transitionBufInd] = acZ;
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

void retrieveCAN()
{
    // TODO: Read messages while CAN available. Write if statements to unpack data dependent upon message id.
    CANFDMessage message;

    // Unpack data frame from sensor board
    // TODO: Add needed conversions or scaling to this function
    if (message.id == CAN_ID_SENSOR_DATA) {    

        acX = bufferToInt16(message.data[0], message.data[1]);
        acY = bufferToInt16(message.data[2], message.data[3]);
        acZ = bufferToInt16(message.data[4], message.data[5]);
        gyX = bufferToInt16(message.data[6], message.data[7]);
        gyY = bufferToInt16(message.data[8], message.data[9]);
        gyZ = bufferToInt16(message.data[10], message.data[11]);
        // TODO: Barometer. This is 6 bytes ([12] - [17]) but idk what this data actually is. Need temperature and pressure.
        magX = bufferToInt16(message.data[18], message.data[19]);
        magY = bufferToInt16(message.data[20], message.data[21]);
        magZ = bufferToInt16(message.data[22], message.data[23]);
        timestamp = bufferToUint32(message.data[25], message.data[26], message.data[27], message.data[28]);
    } 
    
    // Unpack GPS frame from sensor board
    else if (message.id == CAN_ID_GPS) {
        // TODO: Figure out how to unpack GPS data. This has not been implemented on the transmitting side yet in the sensor board project.
    }  

    // Unpack battery voltage from power board
    else if (message.id == CAN_ID_VBATT) {
        vBatt = message.data[0];
    }

    // Construct data packet
    constructPacket();
}

int16_t bufferToInt16(uint8_t upper, uint8_t lower) {
    return lower | upper << 8;
}

uint32_t bufferToUint32(uint8_t data3, uint8_t data2, uint8_t data1, uint8_t data0) {
    return data0 | data1 << 8 | data2 << 16 | data3 << 24;
}

// Construct the data packet
void constructPacket() {

    // TODO: When state estiation is completed, need to update this to include velocity, orientation, etc.

    // Timestamp
    DataPacket[0] = (uint8_t) ((timestamp >> 24) & 0xFFU);
    DataPacket[1] = (uint8_t) ((timestamp >> 16) & 0xFFU);
    DataPacket[2] = (uint8_t) ((timestamp >> 8) & 0xFFU);
    DataPacket[3] = (uint8_t) (timestamp & 0xFFU);

    // State
    DataPacket[4] = (uint8_t) avionicsState;

    // Altitude 
    // TODO: Actually create an altitude variable and then update this. Currently have pressure but need to calculate altitude.
    DataPacket[5] = 0;
    DataPacket[6] = 0;
    DataPacket[7] = 0;
    DataPacket[8] = 0;

    // Temperature
    DataPacket[9] = temperature;

    // Battery voltage
    DataPacket[10] = vBatt;

    // Airbrake actuation percent
    DataPacket[11] = abPct;

    // Acceleration (XYZ)
    DataPacket[12] = (uint8_t) ((acX >> 8) & 0xFFU);
    DataPacket[13] = (uint8_t) (acX & 0xFFU);
    DataPacket[14] = (uint8_t) ((acY >> 8) & 0xFFU);
    DataPacket[15] = (uint8_t) (acY & 0xFFU);
    DataPacket[16] = (uint8_t) ((acZ >> 8) & 0xFFU);
    DataPacket[17] = (uint8_t) (acZ & 0xFFU);

    // Angular rate (XYZ)
    DataPacket[18] = (uint8_t) ((acX >> 8) & 0xFFU);
    DataPacket[19] = (uint8_t) (acX & 0xFFU);
    DataPacket[20] = (uint8_t) ((acY >> 8) & 0xFFU);
    DataPacket[21] = (uint8_t) (acY & 0xFFU);
    DataPacket[22] = (uint8_t) ((acZ >> 8) & 0xFFU);
    DataPacket[23] = (uint8_t) (acZ & 0xFFU);
}

void logData()
{
    // TODO: Write data packet to flash chip
}

void doAirbrakeControls(uint16_t velVert, uint16_t velLat, float alt) {
    // TODO: Implement control algorithm here to compute airbrake actuation value from inputs.
    abPct = 0;
}

// Send CAN frame with data packet
void sendCAN() {
    CANFDMessage message;
    message.id == CAN_ID_DATA_PACKET;
    for (int i = 0 ; i < sizeof(DataPacket); i++) {
        message.data[i] = DataPacket[i];
    }
    const bool ok = can.tryToSend (message) ;
    if (ok) { // TODO: Print statements here are for debugging... could actually handle a failure somehow if we wanted to
      // Serial.print ("Sent: ") ;
    } else {
      // Serial.print ("Send failure") ;
    }
}

void setup()
{
    Serial.begin(9600);
    SPI.begin();
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
                    // airbrakeServo.enable;  // Add this back in when using the stack
                    burnoutTimer.reset();
                    state_start = millis();
                    avionicsState = COAST;
                    // airbrakesTimer.reset();
                    break;
                }
            }
            // Coast Contingency 8 second timeout
            // Burnout wasn't detected after 8 seconds, move into coast contingency
            // if (timeout(8000))
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

            // active airbrakes control
            // Sweep program for test launch
            // Send message to power board to actuate the motor
            // airbrakeSweep();

            // TODO: flesh this out more, need to clarify apogee detection
            if (apogeeDetect())
            {
                // airbrakeServo.setPosition(0); // Need to do this upon state transition
                // TODO: Add some delay on a timer to ensure airbrakes get fully retracted
                // airbrakeServo.disable;  // Add this back in upon state transition when using the stack
                state_start = millis();
                avionicsState = DROGUE_DEPLOY;
            }

            // Wait 30 seconds before moving into DROGUE_DEPLOY state if all else fails
            if (timeout(30000))
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
            if (timeout(10000)) {
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
            // if (timeout(120000)) {
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
            if (timeout(10000)) {
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
            if (timeout(100000))
            {
                Serial.println("Main Descent phase timeout triggered!");
                avionicsState = POSTFLIGHT;
                break;
            }
            break;
        case POSTFLIGHT:
            // datalog slow
            // low power mode other peripherals
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

        // if (counter % 25 == 0) {
        // 	Serial.print("Current state: ");
        // 	Serial.print(avionicsState);
        // 	Serial.print("	Accel val: ");
        // 	Serial.println(accelMag);
        // }
    }
}