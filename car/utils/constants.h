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
const int MPU_ACCEL_SCALE_FACTOR = 16384;
const int MPU_GYRO_SCALE_FACTOR = 131;
const int MPU_REG_SMPLRT_DIV   =  0x19;
const int MPU_REG_USER_CTRL    =  0x6A;
const int MPU_REG_PWR_MGMT_1   =  0x6B;
const int MPU_REG_PWR_MGMT_2   =  0x6C;
const int MPU_REG_CONFIG       =  0x1A;
const int MPU_REG_GYRO_CONFIG  =  0x1B;
const int MPU_REG_ACCEL_CONFIG =  0x1C;
const int MPU_REG_FIFO_EN      =  0x23;
const int MPU_REG_INT_ENABLE   =  0x38;
const int MPU_REG_ACCEL_XOUT_H =  0x3B;
const int MPU_REG_SIGNAL_PATH_RESET  = 0x68;

// General
const double EPS = 1e-6;
const double MAX_ROTATION_ANGLE = 90.0;