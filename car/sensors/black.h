#pragma once

#include "../utils/utils.h"
#include "../utils/constants.h"

class BlackSensor {
public:

    BlackSensor() {}

    void setup(PCF857x* pcf1, int pin, bool inverted) {
        this->pcf1 = pcf1;
        this->pin = pin;
        this->inverted = inverted;
    }

    void read(bool& isBlack) {
        isBlack = (pcf1->read(pin) == LOW);

        if (inverted)
            isBlack = !isBlack;
    }

private:
    PCF857x* pcf1;
    int pin;
    bool inverted;
};