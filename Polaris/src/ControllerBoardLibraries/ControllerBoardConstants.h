const double G = 9.80665; // m/s^2
const int ACCEL_THRESHOLD = 3*G; // m/s^2
const int DESCENT_THRESHOLD = -6; // m/s TODO FIX THIS

const int HZ_10 = 10;
const int HZ_100 = 100;

const int CONVERSION = 1000;
const int SECONDS = 1000;   // scaling factor to convert from seconds to milliseconds
// Set this to the desired Hz for the loop
const int LOOP_FREQUENCY = HZ_100;
const double METER_CONVERSION = 0.3048; // conversion from feet to meters

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

// Airbrake servo pin 
const int SERVO_PIN = 7;