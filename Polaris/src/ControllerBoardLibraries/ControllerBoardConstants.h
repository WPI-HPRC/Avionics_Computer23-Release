const double G = 9.80665;                  // m/s^2
const float ACCEL_THRESHOLD = 3;           // g
const int MAIN_DESCENT_THRESHOLD = 7;      // m/s
const int DROGUE_DESCENT_THRESHOLD = 30;   // m/s
const double PITCH_FRACTION = 0.577350269; // 1/sqrt(3) or tan(30 degrees)
const int LAND_THRESHOLD = 20;             // [m] ceiling for landing height in meters
const int ADC_RESOLUTION = 1024;           // resolution for 10-bit ADC
const float ADC_FULL_SCALE_RANGE = 3.3;    // Maximum voltage read by ADC is 3.3V
const int VOLTAGE_DIVIDER_RATIO = 3;       // Scale factor for reading voltage with voltage divider

const int HZ_10 = 10;
const int HZ_100 = 100;
const int HZ_40 = 40;

const int STARTUP_DELAY = 26; // ms, 25 ms for 1 loop, 1 ms for processing time

const int CONVERSION = 1000;
const int SECONDS = 1000; // scaling factor to convert from seconds to milliseconds

// Set this to the desired Hz for the loop
const int LOOP_FREQUENCY = HZ_40;
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

// Calibration constants
const int IMU_CALIBRATION_ITERS = 1000;
const int AGL_CALIBRATION_ITERS = 2000;

// Pin declarations
const int BLUE_LED = 6;
const int SERVO_PIN = 7;
const int RED_LED = 20;
const int YELLOW_LED = 21;
const int BUZZER_PIN = 8;
const int BUTTON_PIN = 17;
const int VBATT_PIN = 16;