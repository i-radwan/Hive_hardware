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

const int RGHT_DIR1 = 2;
const int RGHT_DIR2 = 3;
const int RGHT_SPED = D1;

// Motors PID 
// const double KP = 55;
// const double KI = 0.5;
// const double KD = 7;
// const double LF = 1; // Left motor factor
// const double RF = 0.7; // Right motor factor
// const double I_LIMIT = 10;
const double KP = 7;
const double KI = 0;
const double KD = 0;
const double LF = 1; // Left motor factor
const double RF = 0.7; // Right motor factor
const double I_LIMIT = 10;

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
const double EPS = 0.1;
const double MAX_ROTATION_ANGLE = 90.0;