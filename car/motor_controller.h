#pragma once

#include "sensors/encoder.h"
#include "utils/utils.h"
#include "utils/constants.h"

class MotorController {

public:

    // ====================
    // Members

    Encoder* en;
    PCF857x* pcf;

    double PWM, RPM;
    double p = 0, i = 0, d = 0, diff = 0, previousDiff = 0, speed = 0, targetSpeed = 0; // ToDo

    // ====================
    // Functions

    MotorController(double kp, double ki, double kd, double initThrottle,
                    int speedPin, int dir1Pin, int dir2Pin) :
                    kp(kp), ki(ki), kd(kd), initThrottle(initThrottle),
                    speedPin(speedPin), dir1Pin(dir1Pin), dir2Pin(dir2Pin) {
        pinMode(speedPin, OUTPUT);
    }

    void update() {
        double current = millis();

        if (current - time < MOTORS_ADJUST_DELTA) {
            return;
        }

        double elapsedTime = (current - time) / 1000.0;
        time = current;

        unsigned long ticks = computeTicks();
        totalDistance += computeDistance(ticks);

        RPM = computeRPM(ticks, elapsedTime);
        PWM = computePWM(RPM, elapsedTime);

        sendMotorSignal(PWM);

        // Increment speed gradually until reaching the targetSpeed
        speed = min(speed + speedIncrementStep, targetSpeed);
    }

    void brake() {
        targetSpeed = speed = 0;
        direction = DIRECTION::FORWARD;

        sendMotorSignal(PWMRANGE); // To brake the motors PWMRANGE, HIGH, HIGH
    }

    void setSpeed(double spd) {
        // Set the direction
        if (spd < 0 && abs(spd) > EPS) {
            direction = DIRECTION::BACKWARD;
        } else {
            direction = DIRECTION::FORWARD;
        }

        spd = abs(spd);

        if (spd < speed) { // Sudden reduction
            targetSpeed = speed = spd;
        } else { // Gradual increment
            targetSpeed = spd;
        }

        if (targetSpeed < EPS) { // To stop anytime the speed drops to zero
            brake();
        }
    }

    double getSpeed() {
        return speed;
    }

    double getRPM() {
        return RPM;
    }

    double getPWM() {
        return PWM;
    }

    void setSpeedIncrementStep(double spdIncrementStep) {
        speedIncrementStep = max(0., spdIncrementStep);
    }

    double getTotalDistance() {
        return totalDistance;
    }

    void reset() {
        p = i = d = 0;
        totalDistance = previousDiff = diff = 0;

        direction = DIRECTION::FORWARD;

        noInterrupts();
        en->reset();
        interrupts();
    }

private:

    // ====================
    // Members

    const double kp, ki, kd, initThrottle;
    const int speedPin, dir1Pin, dir2Pin;

    double speedIncrementStep = MOTORS_MOVE_SPEED_INCREMENT;

    double time = millis();
    double totalDistance = 0;

    DIRECTION direction = DIRECTION::FORWARD;

    // ====================
    // Functions

    double computePWM(double spd, double elapsedTime) {
        if (targetSpeed < EPS) { // If the speed is zero, force stop the motor
            brake();

            return 0;
        }

        diff = speed - spd;

        p = diff * kp;
        i = i + diff * ki;
        d = ((diff - previousDiff) / elapsedTime) * kd;

        previousDiff = diff;

        double pid = constrain(initThrottle + (p + i + d), -PWMRANGE, PWMRANGE);

        double throttle = constrain(pid, 0, PWMRANGE);

        return throttle;
    }

    double computeRPM(double ticks, double elapsedTime) {
        return (ticks / DISK_SLOTS) * float(60) / elapsedTime;
    }

    double computeDistance(unsigned long ticks) {
        return (ticks / DISK_SLOTS) * WHEEL_DIAMETER * M_PI;
    }

    unsigned long computeTicks() {
        unsigned long ticks;

        noInterrupts();
        en->readAndReset(ticks);
        interrupts();

        return ticks;
    }

    void sendMotorSignal(double PWM) {
        int dir1, dir2;

        if (speed < EPS) {
            dir1 = dir2 = HIGH;
        } else if (direction == DIRECTION::FORWARD) {
            dir1 = HIGH;
            dir2 = LOW;
        } else if (direction == DIRECTION::BACKWARD) {
            dir1 = LOW;
            dir2 = HIGH;
        }

        analogWrite(speedPin, PWM);

        pcf->write(dir1Pin, dir1);
        pcf->write(dir2Pin, dir2);
    }
};