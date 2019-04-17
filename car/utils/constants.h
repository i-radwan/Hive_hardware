#pragma once

// WIFI
const char* NET_NAME = "Radwan";
const char* NET_PASS = "9992009a";
const int PORT = 12345;
const char* MSG_ACK = "ACK";

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

const int ULTRA_SONIC_TRIGGER_PIN = D3;
const int ULTRA_SONIC_ECHO_PIN = D5;
const int MIN_DISTANCE = 15; // cm. Min distance ahead of the robot

const int SERVO_PIN = D8;
const int SERVO_DOWN_ANGLE = 20;
const int SERVO_UP_ANGLE = 110;

const int LFT_BLACK_SENSOR_PIN = 4;
const int RGT_BLACK_SENSOR_PIN = 5;

// General
const double MOTORS_ADJUST_DELTA = 150; // ms
const double MOTORS_INIT_SPEED = 60; // rpm
const double MOTORS_MAX_SPEED = 100; // rpm
const double MOTORS_ROTATION_PWM = 150;
const double STEP = 300; // mm

enum STATE {
    INIT,
    IDLE,
    MOVE,
    ROTATE,
    ALIGN
};

enum DIRECTION {
    FWARD,
    BWARD
};