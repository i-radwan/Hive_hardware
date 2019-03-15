#pragma once

#include "communicator.h"
#include "utils/utils.h"
#include "utils/constants.h"

class Navigator {
public:

    void setup(Communicator* comm, PCF857x* pcf1) {
        // Motor pins
        pinMode(LEFT_DIR1, OUTPUT);
        pinMode(LEFT_DIR2, OUTPUT);
        pinMode(LEFT_SPED, OUTPUT);
        pinMode(RGHT_DIR1, OUTPUT);
        pinMode(RGHT_DIR2, OUTPUT);
        pinMode(RGHT_SPED, OUTPUT);

        this->comm = comm;
        this->pcf1 = pcf1;
    }

    void move(double currentAngle) {
        if (movingForward || movingBackward) {
            align(currentAngle);
        }
    }
    
    void stop() {
        movingForward = movingBackward = false;
        adjustMotors(LOW, LOW, LOW, LOW, LOW, LOW);
    }
    
    void forward(double currentAngle) {
        time = millis();
        pidI = 0;
        
        movingBackward = false;
        movingForward = true;
        
        referenceAngle = currentAngle;
        
        adjustMotors(PWMRANGE, PWMRANGE, HIGH, LOW, HIGH, LOW);
    }

    void backward(double currentAngle) {
        time = millis();
        pidI = 0;
        
        movingForward = false;
        movingBackward = true;
        
        referenceAngle = currentAngle;
        
        adjustMotors(PWMRANGE, PWMRANGE, LOW, HIGH, LOW, HIGH);
    }

    void left(double currentAngle) {
        time = millis();
        pidI = 0;
        
        referenceAngle = referenceAngle - 90;

        if (referenceAngle < -180) {
            referenceAngle +=  360;
        }
    }

    void right(double currentAngle) {
        time = millis();
        pidI = 0;
        
        referenceAngle = referenceAngle + 90;

        if (referenceAngle > 180) {
            referenceAngle -= 360;
        }
    }

private:
    Communicator* comm;
    PCF857x* pcf1;

    double time = 0;
    double prevDiff = 0;
    double pid = 0, pidP = 0, pidI = 0, pidD = 0;

    double referenceAngle = 0;
    bool movingForward = false, movingBackward = false;
    
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

        const double throttle = PWMRANGE / 2.0;

        // Apply limits        
        pid = max(pid, -PWMRANGE * 2.0);
        pid = min(pid, PWMRANGE * 2.0);

        double leftPWM = (throttle - pid), rightPWM = (throttle + pid);

        // Apply limits
        leftPWM = max(leftPWM, PWMRANGE * -1.0);
        leftPWM = min(leftPWM, PWMRANGE * 1.0);
        rightPWM = max(rightPWM, PWMRANGE * -1.0);
        rightPWM = min(rightPWM, PWMRANGE * 1.0);

        // Set directions
        int lDir1, lDir2, rDir1, rDir2;

        if (movingForward) {
            lDir1 = HIGH;
            lDir2 = LOW;
            rDir1 = HIGH;
            rDir2 = LOW;
        } else if (movingBackward) {
            lDir1 = LOW;
            lDir2 = HIGH;
            rDir1 = LOW;
            rDir2 = HIGH;

            Utils::swap(&leftPWM, &rightPWM);
        }

        // -ve values mean the other direction
        if (leftPWM < 0) {
            leftPWM = -leftPWM;

            Utils::swap(&lDir1, &lDir2);
        }

        if (rightPWM < 0) {
            rightPWM = -rightPWM;

            Utils::swap(&rDir1, &rDir2);
        }

        prevDiff = diff;

        // this->comm->send(String(diff) + " " + String(pid) + " " + String(leftPWM * LF) + " " + String(rightPWM * RF));

        adjustMotors(leftPWM * LF, rightPWM * RF, lDir1, lDir2, rDir1, rDir2);
    }

    void adjustMotors(int lSpeed, int rSpeed, int lDir1, int lDir2, int rDir1, int rDir2) {
        analogWrite(LEFT_SPED, lSpeed);
        analogWrite(RGHT_SPED, rSpeed);

        pcf1->write(LEFT_DIR1, lDir1);
        pcf1->write(LEFT_DIR2, lDir2);
        pcf1->write(RGHT_DIR1, rDir1);
        pcf1->write(RGHT_DIR2, rDir2);
    }
};