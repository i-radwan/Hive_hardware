#pragma once

#include "../utils/utils.h"
#include "../utils/constants.h"

class BlackSensor {
public:

    BlackSensor() {}

    void setup(PCF857x* pcf1, int pin, bool inverted, bool isConnectedToPCF) {
        this->pcf1 = pcf1;
        this->pin = pin;
        this->inverted = inverted;
        this->isConnectedToPCF = isConnectedToPCF;

        if (!isConnectedToPCF)
            pinMode(pin, INPUT_PULLUP);
    }

    void read(bool& isBlack) {
        if (isConnectedToPCF)
            isBlack = (pcf1->read(pin) == LOW);
        else 
            isBlack = digitalRead(pin);

        if (inverted)
            isBlack = !isBlack;
    }

private:
    PCF857x* pcf1;
    int pin;
    bool inverted;
    bool isConnectedToPCF;
};