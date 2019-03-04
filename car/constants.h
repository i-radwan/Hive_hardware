#pragma once

// Motor pins
int LEFT_DIR1 = D0;
int LEFT_DIR2 = D1;
int LEFT_SPED = D2;

int RGHT_DIR1 = D3;
int RGHT_DIR2 = D4;
int RGHT_SPED = D5;

// WIFI
const char* NET_NAME = "TYT 1";
const char* NET_PASS = "tyt@1111";
const int PORT = 12345;

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
