#pragma once

#include <math.h>

#include "../utils/utils.h"
#include "../utils/constants.h"

class BatterySensor {

public:

    BatterySensor() {}

    void setup(int pin) {
        this->pin = pin;

        pinMode(pin, INPUT);
    }

    void read(uint8_t& batteryLevel) {
        double current = millis();

        if (current - lastRead < BATTERY_SENSOR_REFRESH_RATE) {
            batteryLevel = lastValue;

            return;
        }

        lastRead = current;

        int val = analogRead(pin);

        double remaining = (val / 1024.) * BATTERY_DIVIDED_REF_VOLT * BATTERY_RESISTORS_RATIO;

        remaining = max(0., remaining - BATTERY_EMPTY_REF_VOLT);

        remaining /= (BATTERY_REF_VOLT - BATTERY_EMPTY_REF_VOLT);

        batteryLevel = lastValue = (uint8_t) round(remaining * 10); // [0 - 10] levels
    }

private:

    int pin;
    double lastRead = 0;
    uint8_t lastValue = 0;
};