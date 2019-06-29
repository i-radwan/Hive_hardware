#pragma once

#include "sensors/encoder.h"
#include "utils/utils.h"
#include "utils/constants.h"

struct MotorController {

public:

    // ====================
    // Members

    Encoder* en;
    PCF857x* pcf;

    double p = 0, i = 0, d = 0, diff = 0, prevDiff = 0, speed = 0, targetSpeed = 0;

    // ====================
    // Functions

    MotorController(double kp, double ki, double kd, double spd,
                    int speedPin, int dir1Pin, int dir2Pin) :
                    kp(kp), ki(ki), kd(kd),
                    speedPin(speedPin), dir1Pin(dir1Pin), dir2Pin(dir2Pin) {
        speed = spd;

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

        double RPM = computeRPM(ticks, elapsedTime);
        double PWM = computePWM(RPM, elapsedTime);

        sendMotorSignal(PWM);

        // Increment speed gradually until reaching the targetSpeed
        speed = min(speed + MOTORS_SPEED_INCREMENT, targetSpeed);
    }

    void brake() {
        speed = 0;

        sendMotorSignal(PWMRANGE); // To brake the motors PWMRANGE, HIGH, HIGH
    }

    void setSpeed(double spd) {
        if (spd < speed) { // Sudden reduction
            speed = spd;
        } else { // Gradual increment
            targetSpeed = spd;
        }

        if (abs(speed) < EPS) { // To stop anytime the speed drops to zero
            brake();
        }
    }

    double getSpeed() {
        return speed;
    }

    double getTotalDistance() {
        return totalDistance;
    }

    void reset() {
        p = i = d = 0;
        totalDistance = prevDiff = diff = 0;

        noInterrupts();
        en->reset();
        interrupts();
    }

private:

    // ====================
    // Members

    const double kp, ki, kd;
    const int speedPin, dir1Pin, dir2Pin;

    double time = millis();
    double totalDistance = 0;

    // ====================
    // Functions

    double computePWM(double spd, double elapsedTime) {
        if (speed < EPS) { // If the speed is zero, force stop the motor
            reset();

            return 0;
        }

        diff = abs(speed) - spd;

        p = diff * kp;
        i = i + diff * ki;
        d = ((diff - prevDiff) / elapsedTime) * kd;

        prevDiff = diff;

        double pid = constrain(MOTORS_INIT_THROTTLE + (p + i + d), -PWMRANGE, PWMRANGE);

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

        if (abs(speed) < EPS) {
            dir1 = dir2 = HIGH;
        } else if (speed < 0) {
            dir1 = LOW;
            dir2 = HIGH;
        } else {
            dir1 = HIGH;
            dir2 = LOW;
        }

        analogWrite(speedPin, PWM);

        pcf->write(dir1Pin, dir1);
        pcf->write(dir2Pin, dir2);
    }
};
