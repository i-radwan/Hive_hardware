#pragma once

#include "../utils/utils.h"
#include "../utils/constants.h"

class BlackSensor {
public:

    BlackSensor() {}

    void setup(PCF857x* pcf1, int pin) {
        this->pcf1 = pcf1;
        this->pin = pin;
    }

    void read(bool& isBlack) {
        isBlack = (pcf1->read(pin) == LOW);
    }

private:
    PCF857x* pcf1;
    int pin;
};