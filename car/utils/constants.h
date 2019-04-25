#pragma once

// WIFI
const char* NET_NAME = "Radwan";
const char* NET_PASS = "9992009a";
const int PORT = 12345;
const String MSG_SET = "0";
const String MSG_ACK = "1";
const String MSG_ERR = "2";

// Serial
const int BAUD_RATE = 115200;

// Communication
enum MSG {
    NONE = -1,
    STOP = 0,
    FORWARD = 1,
    BACKWARD = 2,
    LEFT = 3,
    RIGHT = 4,
    UP = 5,
    DOWN = 6
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

const double DISK_SLOTS = 20.0;
const int WHEEL_DIAMETER = 67; // mm
const int DEBOUNCE_DELTA = 1000; // us

// PCF8574
const int PCF1_ADDRESS = 0x20;

// Sensors
const int MPU_ADDRESS = 0x68;
const int I2C_SCL = D6;
const int I2C_SDA = D7;
const int MPU_INTR = D8;
const double MPU_ACCEL_X_OFF = -24280 / 8.8;
const double MPU_ACCEL_Y_OFF = -1100 / 6.3;
const double MPU_ACCEL_Z_OFF = 7000 / 6.9;
const double MPU_GYRO_X_OFF = -78 / 3.5;
const double MPU_GYRO_Y_OFF = -391 / 4.05;
const double MPU_GYRO_Z_OFF = -185 / 3.5;

const int OPTICAL_CLCK = 1;
const int OPTICAL_DATA = 3;
const double OPTICAL_DPI_TO_MM = 1 / 39.0;

const int ULTRA_SONIC_TRIGGER_PIN = D9;
const int ULTRA_SONIC_ECHO_PIN = D5;
const int MIN_DISTANCE = 15; // cm. Min distance ahead of the robot

const int SERVO_PIN = D3;
const int SERVO_DOWN_ANGLE = 20;
const int SERVO_UP_ANGLE = 110;

const int FRT_LFT_BLACK_SENSOR_PIN = 6;
const int FRT_RGT_BLACK_SENSOR_PIN = 7;
const int FRT_CNT_BLACK_SENSOR_PIN = D8;
const int BAK_LFT_BLACK_SENSOR_PIN = 4;
const int BAK_RGT_BLACK_SENSOR_PIN = 5;

const bool FRT_LFT_BLACK_SENSOR_INV = false;
const bool FRT_RGT_BLACK_SENSOR_INV = false;
const bool FRT_CNT_BLACK_SENSOR_INV = false;
const bool BAK_LFT_BLACK_SENSOR_INV = true;
const bool BAK_RGT_BLACK_SENSOR_INV = true;

const bool FRT_LFT_BLACK_SENSOR_PCF = true;
const bool FRT_RGT_BLACK_SENSOR_PCF = true;
const bool FRT_CNT_BLACK_SENSOR_PCF = false;
const bool BAK_LFT_BLACK_SENSOR_PCF = true;
const bool BAK_RGT_BLACK_SENSOR_PCF = true;

// General
const double MOTORS_ADJUST_DELTA = 150; // ms
const double MOTORS_INIT_SPEED = 45; // rpm
const double MOTORS_MAX_SPEED = 80; // rpm
const double MOTORS_ROTATION_SPEED = 35;
const double STEP = 300; // mm

enum STATE {
    INIT,               // 0
    IDLE,               // 1
    FWARD,              // 2
    STRAIGHT,           // 3
    STRAIGHT_LEFT,      // 4
    STRAIGHT_RIGHT,     // 5
    OFFLINE_LEFT,       // 6
    OFFLINE_RIGHT,      // 7
    ROTATE_RIGHT,       // 8
    ROTATE_LEFT,        // 9
    PRE_ROTATE_RIGHT,   // 10
    PRE_ROTATE_LEFT,    // 11
    PRE_ROTATE_RIGHT_2, // 12
    PRE_ROTATE_LEFT_2,  // 13
    POST_ROTATE_RIGHT,  // 14
    POST_ROTATE_LEFT,   // 15
    ALIGNMENT           // 16
};