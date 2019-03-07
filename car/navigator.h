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
    }

    void move(double currentAngle) {
        if (movingForward || movingBackward || rotatingLeft || rotatingRight) {
            align(currentAngle);
        }
    }
    
    void stop() {
        Serial.println("Stopping...");
        movingForward = movingBackward = false;
        rotatingLeft = rotatingRight = false;
        adjustMotors(LOW, LOW, LOW, LOW, LOW, LOW);
    }
    
    void forward(double currentAngle) {
        time = millis();
        pidI = 0;
        movingBackward = rotatingRight = rotatingLeft = false;
        movingForward = true;
        referenceAngle = currentAngle;
        adjustMotors(PWMRANGE, PWMRANGE, HIGH, LOW, HIGH, LOW);
    }

    void backward(double currentAngle) {
        time = millis();
        pidI = 0;
        movingForward = rotatingRight = rotatingLeft = false;
        movingBackward = true;
        referenceAngle = currentAngle;
        adjustMotors(PWMRANGE, PWMRANGE, LOW, HIGH, LOW, HIGH);
    }

    void left(double currentAngle) {
        time = millis();
        pidI = 0;
        movingForward = movingBackward = rotatingRight = false;
        rotatingLeft = true;

        referenceAngle = prevRotationAngle = currentAngle - 90;

        if (referenceAngle < -180) {
            referenceAngle +=  360;
        }
    }

    void right(double currentAngle) {
        time = millis();
        pidI = 0;
        movingForward = movingBackward = rotatingLeft = false;
        rotatingRight = true;
        
        referenceAngle = prevRotationAngle = currentAngle + 90;

        if (referenceAngle > 180) {
            referenceAngle -= 360;
        }
    }

    double getReferenceAngle() {
        return referenceAngle;
    }

private:
    double time = 0;
    double prevDiff = 0;
    double pid = 0, pidP = 0, pidI = 0, pidD = 0;

    double referenceAngle = 0;
    double prevRotationAngle = 0;
    bool rotatingLeft = false, rotatingRight = false, movingForward = false, movingBackward = false;

    bool rotate(double currentAngle, bool left, bool right) {
        double diff = currentAngle - referenceAngle;

        if ((left && prevRotationAngle > referenceAngle && currentAngle < referenceAngle) ||
            (right && prevRotationAngle < referenceAngle && currentAngle > referenceAngle)) {
            return true;
        }

        if (left) {
            adjustMotors(LOW, PWMRANGE, LOW, LOW, HIGH, LOW);
        } else if (right) {
            adjustMotors(PWMRANGE, LOW, HIGH, LOW, LOW, LOW);
        }

        prevRotationAngle = currentAngle;

        return false;
    }
    
    void align(double currentAngle) {
        // Update timers
        double current = millis();
        double elapsedTime = (current - time) / 1000.0;
        time = current;

        // PID calculations
        double diff = currentAngle - referenceAngle;
        if (diff > 180) {
            diff -= 360;
        }

        if (diff < -180) {
            diff += 360;
        }
        
        pidP = KP * diff;
        pidI += (diff < I_LIMIT && diff > -I_LIMIT) ? (KI * diff) : 0;
        pidD = KD * ((diff - prevDiff) / elapsedTime);
        pid = pidP + pidI + pidD;

        // Apply limits        
        pid = max(pid, -PWMRANGE * 1.0);
        pid = min(pid, PWMRANGE * 1.0);

        double leftPWM = (PWMRANGE - pid), rightPWM = (PWMRANGE + pid);

        // Apply limits
        leftPWM = max(leftPWM, 0.0);
        leftPWM = min(leftPWM, PWMRANGE * 1.0);
        rightPWM = max(rightPWM, 0.0);
        rightPWM = min(rightPWM, PWMRANGE * 1.0);

        prevDiff = diff;

        if (movingForward || rotatingRight || rotatingLeft)
            adjustMotors(leftPWM * LF, rightPWM * RF, HIGH, LOW, HIGH, LOW);
        else if (movingBackward)
            adjustMotors(rightPWM * LF, leftPWM * RF, LOW, HIGH, LOW, HIGH);
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