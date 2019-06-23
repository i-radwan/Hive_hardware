#pragma once

#include "../utils/utils.h"
#include "../utils/constants.h"

class UltrasonicSensor {

private:
    double lastRead = millis();

public:

    UltrasonicSensor() {}

    void setup() {
        // Sensor pins
        pinMode(ULTRA_SONIC_TRIGGER_PIN, FUNCTION_3);
        pinMode(ULTRA_SONIC_TRIGGER_PIN, OUTPUT);
        pinMode(ULTRA_SONIC_ECHO_PIN, INPUT);
    }

    void read(double& distance) {
        double current = millis();

        if (current - lastRead < ULTRA_SONIC_REFRESH_TIME) {
            return;
        }

        lastRead = current;

        // Clears the trigger pin
        digitalWrite(ULTRA_SONIC_TRIGGER_PIN, LOW);
        delayMicroseconds(2);

        // Sets the ULTRA_SONIC_TRIGGER_PIN on HIGH state for 10 micro seconds
        digitalWrite(ULTRA_SONIC_TRIGGER_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRA_SONIC_TRIGGER_PIN, LOW);

        // Reads the ULTRA_SONIC_ECHO_PIN, returns the sound wave travel time in microseconds
        double sentTime = micros();
        unsigned long duration = pulseIn(ULTRA_SONIC_ECHO_PIN, HIGH, ULTRA_SONIC_TIMEOUT);

        // Calculating the cm distance (via sound speed 343 m/s == 0.034 cm/microsecond)
        distance = duration * 0.034 / 2;

        if (micros() - sentTime >= ULTRA_SONIC_TIMEOUT) // In case pulseIn timed out, we don't want a zero value, this will stop the robot.
            distance = MIN_DISTANCE + 1;
    }
};