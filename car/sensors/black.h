#pragma once

#include "../utils/utils.h"
#include "../utils/constants.h"

class BlackSensor {
public:

    BlackSensor() {}

    void setup(PCF857x* pcf1) {
        this->pcf1 = pcf1;
    }

    void read(bool& isBlack) {
        isBlack = (this->pcf1->read(BLACK_SENSOR_PIN) == LOW);
    }

private:
    PCF857x* pcf1;
};