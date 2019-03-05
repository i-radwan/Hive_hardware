#pragma once

#include "utils/constants.h"

class Navigator {
public:

    void setup() {      
        // Motor pins
        pinMode(LEFT_DIR1, OUTPUT);
        pinMode(LEFT_DIR2, OUTPUT);
        pinMode(LEFT_SPED, OUTPUT);
        pinMode(RGHT_DIR1, OUTPUT);
        pinMode(RGHT_DIR2, OUTPUT);
        pinMode(RGHT_SPED, OUTPUT);
    
        // Enable the two motors
        adjustMotors(PWMRANGE, PWMRANGE, LOW, LOW, LOW, LOW);
    }

    void move(double currentAngle) {
        if (rotating) {
            rotating = !rotate(currentAngle);
            
            if (!rotating)
                stop();
        } else if (moving) {
            align(currentAngle);
        }
    }
    
    void stop() {
        Serial.println("Stopping...");
        moving = false;
        rotating = false;
        adjustMotors(LOW, LOW, LOW, LOW, LOW, LOW);
    }
    
    void forward() {
        Serial.println("Forward...");
        moving = true;
        adjustMotors(PWMRANGE, PWMRANGE, HIGH, LOW, HIGH, LOW);
    }

    void backward() {
        Serial.println("Backward...");
        moving = true;
        adjustMotors(PWMRANGE, PWMRANGE, LOW, HIGH, LOW, HIGH);
    }

    void left() {
        Serial.println("Left...");
        moving = false;
        rotating = true;
        adjustReferenceAngle(-90);
    }

    void right() {
        Serial.println("Right...");
        moving = false;
        rotating = true;
        adjustReferenceAngle(90);
    }

private:
    double referenceAngle = 0;
    bool rotating = false, moving = false;

    bool rotate(double currentAngle) {
        double diff = currentAngle - referenceAngle;

        Serial.println(diff);

        if (abs(diff) < EPS) {
            return true;
        }

        if (currentAngle > referenceAngle) {
            adjustMotors(LOW, PWMRANGE, LOW, LOW, HIGH, LOW);
        } else {
            adjustMotors(PWMRANGE, LOW, HIGH, LOW, LOW, LOW);
        }

        return false;
    }
    
    void align(double currentAngle) {
        double diff = currentAngle - referenceAngle;

        if (abs(diff) < EPS) {
            return;
        }

        if (currentAngle > referenceAngle) {
            adjustMotors((diff / MAX_ROTATION_ANGLE) * PWMRANGE, PWMRANGE, HIGH, LOW, HIGH, LOW);
        } else {
            adjustMotors(PWMRANGE, (-diff / MAX_ROTATION_ANGLE) * PWMRANGE, HIGH, LOW, HIGH, LOW);
        }
    }

    void adjustReferenceAngle(double angle) {
        referenceAngle += angle;
    }

    void adjustMotors(int lSpeed, int rSpeed, int lDir1, int lDir2, int rDir1, int rDir2) {
        Serial.println("Adjust motors..." + (String) lSpeed + " "  + (String) rSpeed + " "  + 
            (String) lDir1 + " "  + (String) lDir2 + " "  + (String) rDir1 + " "  + 
            (String) rDir2);

        analogWrite(LEFT_SPED, lSpeed);
        analogWrite(RGHT_SPED, rSpeed);
        digitalWrite(LEFT_DIR1, lDir1);
        digitalWrite(LEFT_DIR2, lDir2);
        digitalWrite(RGHT_DIR1, rDir1);
        digitalWrite(RGHT_DIR2, rDir2);
    }
};