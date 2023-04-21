const double G = 9.80665;                  // m/s^2
const float ACCEL_THRESHOLD = 3;           // g
const int MAIN_DESCENT_THRESHOLD = 7;     // m/s TODO FIX THIS
const int DROGUE_DESCENT_THRESHOLD = 30;   // m/s TODO FIX THIS
const double PITCH_FRACTION = 0.577350269; // 1/sqrt(3) or tan(30 degrees)
const int LAND_THRESHOLD = 20;             // [m] ceiling for landing height in meters

const float HZ_10 = 10.0;
const float HZ_20 = 20.0;
const float HZ_40 = 40.0;
const float HZ_100 = 100.0;

const float CONVERSION = 1000.0;
const int SECONDS = 1000; // scaling factor to convert from seconds to milliseconds
// Set this to the desired Hz for the loop
const float LOOP_FREQUENCY = HZ_40;
const double METER_CONVERSION = 0.3048; // conversion from feet to meters

// Timer constants
const uint16_t PRELAUNCH_INTERVAL = 1 * SECONDS;
const uint16_t BOOST_MIN_LENGTH = 3 * SECONDS;
const uint16_t COAST_MIN_LENGTH = 10 * SECONDS;

// Timeout values
const uint16_t BOOST_TIMEOUT = 8 * SECONDS;
const uint16_t COAST_TIMEOUT = 30 * SECONDS;
const uint16_t DROGUE_DEPLOY_TIMEOUT = 10 * SECONDS;
const uint32_t DROGUE_DESCENT_TIMEOUT = 120 * SECONDS;
const uint16_t MAIN_DEPLOY_TIMEOUT = 10 * SECONDS;
const uint32_t MAIN_DESCENT_TIMEOUT = 100 * SECONDS;

// Pin declarations
const int BLUE_LED = 6;
const int SERVO_PIN = 7;
const int RED_LED = 20;
const int YELLOW_LED = 21;
const int BUZZER_PIN = 8;
const int BUTTON_PIN = 17;
const int VBATT_PIN = 16;