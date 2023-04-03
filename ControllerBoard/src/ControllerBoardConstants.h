const int ACCEL_THRESHOLD = 3; // m/s^2 TODO: FIX THIS, should be 3G's

const int HZ_10 = 10;
const int HZ_100 = 100;

const int CONVERSION = 1000;
const int SECONDS = 1000;   // scaling factor to convert from seconds to milliseconds
// Set this to the desired Hz for the loop
const int LOOP_FREQUENCY = HZ_100;

// Timer constants
const uint16_t PRELAUNCH_INTERVAL = 1*SECONDS;
const uint16_t BOOST_MIN_LENGTH = 3*SECONDS;

// Timeout values
const uint16_t BOOST_TIMEOUT = 8*SECONDS;
const uint16_t COAST_TIMEOUT = 30*SECONDS;
const uint16_t DROGUE_DEPLOY_TIMEOUT = 10*SECONDS;
const uint32_t DROGUE_DESCENT_TIMEOUT = 120*SECONDS;
const uint16_t MAIN_DEPLOY_TIMEOUT = 10*SECONDS;
const uint32_t MAIN_DESCENT_TIMEOUT = 100*SECONDS;

// CAN frame IDs
const uint8_t CAN_ID_SENSOR_DATA = 0x01;
const uint8_t CAN_ID_GPS = 0x02;
const uint8_t CAN_ID_DATA_PACKET = 0x03;
const uint8_t CAN_ID_VBATT = 0x04;