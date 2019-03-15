#pragma once

#include "PS2Mouse.h"

#include "../utils/utils.h"
#include "../utils/constants.h"

class OpticalSensor {
public:

    OpticalSensor() {}

    void setup() {
        // Sensor pins
        pinMode(OPTICAL_CLCK, FUNCTION_3);
        pinMode(OPTICAL_DATA, FUNCTION_3);

        // Initialize sensor
        mouse = new PS2Mouse(OPTICAL_CLCK, OPTICAL_DATA);
        
        mouse->begin();
    }

    void read(uint8_t& stat, double& dx, double& dy) {
        int x, y;
        mouse->getPosition(stat, x, y);

        dx = x;
        dy = y;

        bool negx = bitRead(stat, 4);
        bool negy = bitRead(stat, 5);

        if (negx)
            dx = Utils::twosToDecimal(dx);

        if (negy)
            dy = Utils::twosToDecimal(dy);

        dx *= OPTICAL_DPI_TO_MM;
        dy *= OPTICAL_DPI_TO_MM;
    }

private:
    PS2Mouse* mouse;
};