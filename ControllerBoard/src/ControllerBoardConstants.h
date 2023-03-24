const int ACCEL_THRESHOLD = 10; // m/s^2

const int HZ_10 = 10;
const int HZ_100 = 100;

const int CONVERSION = 1000;
// Set this to the desired Hz for the loop
const int LOOP_FREQUENCY = HZ_100;

const int MOTOR_BURNOUT_INTERVAL = 0;
const int PRELAUNCH_INTERVAL = 1000; // 1 second
const int BOOST_MIN_LENGTH = 3000; // 3 seconds

// CAN frame IDs
const int CAN_ID_SENSOR_DATA = 0;
const int CAN_ID_GPS = 1;
const int CAN_ID_VBATT = 2;
const int CAN_ID_DATA_PACKET = 3;