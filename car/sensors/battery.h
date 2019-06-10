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

    void read(bool& isLow) {
        int val = analogRead(pin);

        double remaining = (val / 1023) * BATTERY_DIVIDED_REF_VOLT * BATTERY_RESISTORS_RATIO;

        isLow = remaining < (BATTERY_REF_VOLT * BATTERY_WARNING_PERCENTAGE);
    }

private:
    int pin;
};