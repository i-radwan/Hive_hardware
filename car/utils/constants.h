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
    RIGHT = 4
};

// Motors pins
const int LEFT_DIR1 = 0;
const int LEFT_DIR2 = 1;
const int LEFT_SPED = D0;

const int RGHT_DIR1 = 3;
const int RGHT_DIR2 = 2;
const int RGHT_SPED = D1;

// Motors PID
const double KP = 0.65;
const double KI = 0;
const double KD = 1;
const double I_LIMIT = 3;

const double KP2 = 3;
const double KI2 = 0.1;
const double KD2 = 1;
const double I_LIMIT2 = 3;

// Motors encoders
const int LEFT_ENC = D2;
const int RGHT_ENC = D5;

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

// General
const double MOTORS_ADJUST_DELTA = 200; // ms
const double MOTORS_INIT_SPEED = 50; // rpm
const double MOTORS_MAX_SPEED = 55; // rpm
const double MOTORS_ROTATION_PWM = 200;
const double STEP = 260; // mm

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