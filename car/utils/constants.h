#pragma once

// Motor pins
int LEFT_DIR1 = D0;
int LEFT_DIR2 = D1;
int LEFT_SPED = D2;

int RGHT_DIR1 = D3;
int RGHT_DIR2 = D4;
int RGHT_SPED = D5;

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

// Sensors
const int MPU_ADDRESS = 0x68;
const int MPU_I2C_SCL = D6;
const int MPU_I2C_SDA = D7;
const int MPU_INTR = D8;
const double MPU_ACCEL_X_OFF = -24280 / 8.8;
const double MPU_ACCEL_Y_OFF = -1100 / 6.3;
const double MPU_ACCEL_Z_OFF = 7000 / 6.9;
const double MPU_GYRO_X_OFF = -78 / 3.5;
const double MPU_GYRO_Y_OFF = -391 / 4.05;
const double MPU_GYRO_Z_OFF = -185 / 3.5;

// General
const double EPS = 0.01;
const double MAX_ROTATION_ANGLE = 90.0;