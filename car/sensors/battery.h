#pragma once

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
        int val = analogRead(pin);

        double remaining = (val / 1023) * BATTERY_DIVIDED_REF_VOLT * BATTERY_RESISTORS_RATIO;

        batteryLevel = (uint8_t) ((remaining - EPS) / BATTERY_REF_VOLT) * 10; // 0 - 9 levels
    }

private:
    int pin;
};