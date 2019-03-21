#pragma once

#include "../utils/utils.h"
#include "../utils/constants.h"

class Encoder {
public:

    void setup(int pin) {
    	pinMode(pin, INPUT);
    }

	void ticksHandler() {
		unsigned long current = micros();
		if (current - debounce > DEBOUNCE_DELTA) {
		    debounce = current;
			this->ticks++;
	    }
	}

    void read(unsigned long& ticks) {
		ticks = this->ticks / 2;
    }

    void readAndReset(unsigned long& ticks) {
		ticks = this->ticks / 2;
		this->ticks = 0;
    }

	void reset() {
		this->ticks = 0;
	}

private:
    volatile unsigned long ticks = 0;
    volatile unsigned long debounce = 0;
};