#pragma once

#define CAR1
// #define CAR2
// #define CAR3

// PWM
const int PWM_FREQUENCY = 1; // In range 1-1000 KHz

// WIFI
const char* NET_NAME = "Ibrahim's iPhone";
const char* NET_PASS = "20061996";
const char* SERVER = "172.20.10.5";
const int PORT = 12345;
const int WS_PORT = 12344;
const int RECONNECT_INTERVAL = 5000; // ms
const int PING_INTERVAL = 100; // ms
const int PONG_TIMEOUT = 1000; // ms
const int RETRIES_COUNT = 2; // ms

// Serial
const int BAUD_RATE = 115200;

//
// Communication
//

// From server
enum class MSG_FROM_SERVER {
    CONFIG = 0,
    ACTION = 1,
    LIGHTS = 2
};

enum class ACTIONS {
    STOP = 0,
    MOVE = 1,
    ROTATE_RIGHT = 2,
    ROTATE_LEFT = 3,
    RETREAT = 4,
    LOAD = 5,
    OFFLOAD = 6
};

enum class LIGHTS {
    RED = 0,
    BLUE = 1
};

enum class LIGHT_MODE {
    OFF = 0,
    ON = 1,
    FLASH = 2
};

// To server
enum class MSG_TO_SERVER {
    DONE = 0,
    BATTERY = 1,
    BLOCKING = 2,
    ERROR = 3
};

enum class BLOCKING_MODE {
    UNBLOCKED = 0,
    BLOCKED = 1
};

enum class SERVER_TASKS {
    OTHER = -1,
    CONFIG = 0,
    STOP = 1,
    MOVE = 2,
    ROTATE_RIGHT = 3,
    ROTATE_LEFT = 4,
    RETREAT = 5,
    LOAD = 6,
    OFFLOAD = 7,
    RED_LED_OFF = 8,
    RED_LED_ON = 9,
    RED_LED_FLASH = 10,
    BLUE_LED_OFF = 11,
    BLUE_LED_ON = 12,
    BLUE_LED_FLASH = 13
};

// Motors pins
const int LEFT_DIR1 = 0;
const int LEFT_DIR2 = 1;
const int LEFT_SPED = D4;

const int RGHT_DIR1 = 2;
const int RGHT_DIR2 = 3;
const int RGHT_SPED = D0;

// Motors PID
const double KP = 1;
const double KI = 0;
const double KD = 0.1;
const double I_LIMIT = 3;

const double KP3 = 5;
const double KI3 = 0.1;
const double KD3 = 0;

const double KP2 = 2;
const double KI2 = 0.01;
const double KD2 = 0.5;
const double I_LIMIT2 = 3;

// Motors encoders
const int LEFT_ENC = D1;
const int RGHT_ENC = D2;

const double DISK_SLOTS = 12.0;
const int WHEEL_DIAMETER = 67; // mm
const int DEBOUNCE_DELTA = 1000; // us

// PCF8574
const int PCF1_ADDRESS = 0x20;

// Sensors
const int MPU_ADDRESS = 0x68;
const int I2C_SCL = D6;
const int I2C_SDA = D7;
const int MPU_INTR = D8;

#ifdef CAR1
const double MPU_ACCEL_X_OFF = -24280 / 8.8;
const double MPU_ACCEL_Y_OFF = -1100 / 6.3;
const double MPU_ACCEL_Z_OFF = 7000 / 6.9;
const double MPU_GYRO_X_OFF = -78 / 3.5;
const double MPU_GYRO_Y_OFF = -391 / 4.05;
const double MPU_GYRO_Z_OFF = -185 / 3.5;
#endif

#ifdef CAR2
const double MPU_ACCEL_X_OFF = 3810 / 8.8;
const double MPU_ACCEL_Y_OFF = 4499.03 / 6.3;
const double MPU_ACCEL_Z_OFF = 7130 / 6.9;
const double MPU_GYRO_X_OFF = 145.85 / 3.5;
const double MPU_GYRO_Y_OFF = -175.27 / 4.05;
const double MPU_GYRO_Z_OFF = -32 / 3.5;
#endif

#ifdef CAR3
const double MPU_ACCEL_X_OFF = -17530 / 8.8;
const double MPU_ACCEL_Y_OFF = 200 / 6.3;
const double MPU_ACCEL_Z_OFF = 9806 / 6.9;
const double MPU_GYRO_X_OFF = 342.55 / 3.5;
const double MPU_GYRO_Y_OFF = 144.27 / 4.05;
const double MPU_GYRO_Z_OFF = -141 / 3.5;
#endif

const int OPTICAL_CLCK = 1;
const int OPTICAL_DATA = 3;
const double OPTICAL_DPI_TO_MM = 1 / 39.0;

const int ULTRA_SONIC_TRIGGER_PIN = D9;
const int ULTRA_SONIC_ECHO_PIN = D5;
const int ULTRA_SONIC_REFRESH_TIME = 100; // ms
const int ULTRA_SONIC_TIMEOUT = 2000; // us ... 2cm * 2 = 40cm = ~0.5m takes the sound 300 m/s -> 0.001666666667 seconds to travel = 1666.667us
const int MIN_DISTANCE = 20; // cm. Min distance ahead of the robot

const int SERVO_PIN = D3;
const int SERVO_DOWN_ANGLE = 20;
const int SERVO_UP_ANGLE = 110;

const int BATTERY_SENSOR_PIN = A0;
const double BATTERY_REF_VOLT = 12.4;
const double BATTERY_DIVIDED_REF_VOLT = 3.1; // Divided by 15kΩ & [5kΩ]
const double BATTERY_WARNING_PERCENTAGE = 0.2;
const double BATTERY_RESISTORS_RATIO = 20 / 5; // (15 + 5) / 5

const int FRT_LFT_BLACK_SENSOR_PIN = 6;
const int FRT_RGT_BLACK_SENSOR_PIN = 7;
const int FRT_CNT_BLACK_SENSOR_PIN = D8;
const int BAK_LFT_BLACK_SENSOR_PIN = 4;
const int BAK_RGT_BLACK_SENSOR_PIN = 5;

// Car 1 --> IP : .3
#ifdef CAR1
const bool FRT_LFT_BLACK_SENSOR_INV = false;
const bool FRT_RGT_BLACK_SENSOR_INV = false;
const bool FRT_CNT_BLACK_SENSOR_INV = false;
const bool BAK_LFT_BLACK_SENSOR_INV = true;
const bool BAK_RGT_BLACK_SENSOR_INV = true;
#endif

// Car 2 --> IP : .2
#ifdef CAR2
const bool FRT_LFT_BLACK_SENSOR_INV = true;
const bool FRT_RGT_BLACK_SENSOR_INV = true;
const bool FRT_CNT_BLACK_SENSOR_INV = false;
const bool BAK_LFT_BLACK_SENSOR_INV = false;
const bool BAK_RGT_BLACK_SENSOR_INV = false;
#endif

// Car 3 --> IP : .4
#ifdef CAR3
const bool FRT_LFT_BLACK_SENSOR_INV = true;
const bool FRT_RGT_BLACK_SENSOR_INV = true;
const bool FRT_CNT_BLACK_SENSOR_INV = false;
const bool BAK_LFT_BLACK_SENSOR_INV = false;
const bool BAK_RGT_BLACK_SENSOR_INV = false;
#endif

const bool FRT_LFT_BLACK_SENSOR_PCF = true;
const bool FRT_RGT_BLACK_SENSOR_PCF = true;
const bool FRT_CNT_BLACK_SENSOR_PCF = false;
const bool BAK_LFT_BLACK_SENSOR_PCF = true;
const bool BAK_RGT_BLACK_SENSOR_PCF = true;

const int BATTERY_LEVEL_SIZE = 10;  // Each level represents 10% of the battery life.

// LEDs
const int BLUE_LED_PIN = D3;
const int RED_LED_PIN = D10;
const int FLASH_PERIOD = 500; // ms

// General
const double MOTORS_ADJUST_DELTA = 100; // ms
const double MOTORS_INIT_SPEED = 25; // rpm
const double MOTORS_MAX_SPEED = 30; // rpm
const double MOTORS_ROTATION_SPEED = 15;
const double STEP = 300; // mm
const double EPS = 1e-6;
const double MPU_REFRESH_RATE = 0; // ms

enum STATE {
    INIT,                       // 0
    IDLE,                       // 1
    MOVE,                       // 2
    STRAIGHT,                   // 3
    STRAIGHT_LEFT,              // 4
    STRAIGHT_RIGHT,             // 5
    OFFLINE_LEFT,               // 6
    OFFLINE_RIGHT,              // 7
    ROTATE_RIGHT,               // 8
    ROTATE_LEFT,                // 9
    PRE_ROTATE_RIGHT,           // 10
    PRE_ROTATE_LEFT,            // 11
    PRE_ROTATE_RIGHT_2,         // 12
    PRE_ROTATE_LEFT_2,          // 13
    POST_ROTATE_RIGHT,          // 14
    POST_ROTATE_LEFT,           // 15
    ALIGNMENT,                  // 16 ToDo: remove this not-used state
    PRE_RETREAT,                // 17
    RETREAT,                    // 18
    RETREAT_LEFT,               // 19
    RETREAT_RIGHT,              // 20
    RETREAT_OFFLINE_LEFT,       // 21
    RETREAT_OFFLINE_RIGHT       // 22
};

