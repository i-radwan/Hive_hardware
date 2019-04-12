#pragma once

#include "../utils/utils.h"
#include "../utils/constants.h"

class UltrasonicSensor {
public:

    UltrasonicSensor() {}

    void setup() {
        // Sensor pins
        pinMode(ULTRA_SONIC_TRIGGER_PIN, OUTPUT);
        pinMode(ULTRA_SONIC_ECHO_PIN, INPUT);
    }

    void read(double& distance) {
        // Clears the trigger pin
        digitalWrite(ULTRA_SONIC_TRIGGER_PIN, LOW);
        delayMicroseconds(2);

        // Sets the ULTRA_SONIC_TRIGGER_PIN on HIGH state for 10 micro seconds
        digitalWrite(ULTRA_SONIC_TRIGGER_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRA_SONIC_TRIGGER_PIN, LOW);

        // Reads the ULTRA_SONIC_ECHO_PIN, returns the sound wave travel time in microseconds
        unsigned long duration = pulseIn(ULTRA_SONIC_ECHO_PIN, HIGH);

        // Calculating the distance (via sound speed 343 m/s)
        distance = duration * 0.034 / 2;
    }
};