#pragma once

#include "communicator.h"
#include "sensors/encoder.h"
#include "utils/utils.h"
#include "utils/constants.h"

class Navigator {
public:

    // 1.2, 0.1, 0.4
    Navigator() : leftMotorPID(1.3, 0.1, 0.4, MOTORS_INIT_SPEED, 0), 
                  rightMotorPID(1.2, 0.1, 0.3, MOTORS_INIT_SPEED, 0) {
    }

    void setup(Communicator* com, PCF857x* pcf1, Encoder* len, Encoder* ren) {
        // Motor pins
        pinMode(LEFT_DIR1, OUTPUT);
        pinMode(LEFT_DIR2, OUTPUT);
        pinMode(LEFT_SPED, OUTPUT);
        pinMode(RGHT_DIR1, OUTPUT);
        pinMode(RGHT_DIR2, OUTPUT);
        pinMode(RGHT_SPED, OUTPUT);

        this->com = com;
        this->pcf1 = pcf1;
        this->len = len;
        this->ren = ren;
    }

    void init(double currentAngle) {
        if (state != INIT)
            return;

        angle = currentAngle;

        state = IDLE;
    }

    void navigate(double currentAngle, unsigned long distance, bool isLeftBlack, bool isRightBlack) {
        if (state == MOVE)
            move(currentAngle, distance, isLeftBlack, isRightBlack);
        // else if (state == ROTATE)
        //     rotate(currentAngle);
        // else if (state == ALIGN)
        //     align(currentAngle);
    }
    
    void stop(double currentAngle) {
        if (state == INIT) {
            return;
        } else if (state == IDLE) { // ToDo: remove, this is used to reset the angle on double stop commands
            angle = currentAngle;
        }

        state = IDLE;

        adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);
    }
    
    void forward(double currentAngle) {
        init(currentAngle);

        time = millis();
        pidI = 0;
        leftMotorPID.reset();
        rightMotorPID.reset();
        state = MOVE;

        noInterrupts();
        len->reset();
        ren->reset();
        interrupts();

        distance += STEP;
    }

    void backward(double currentAngle) {
        // init(currentAngle);
        
        // time = millis();
        // pidI = 0;
    }

    void left(double currentAngle) {
        init(currentAngle);
        
        time = millis();
        pidI = 0;
        throttle = MOTORS_ROTATION_PWM;
        i = 0;
        prevDiff = currentAngle - angle;
        state = ROTATE;
        
        angle += -90;

        if (angle < -180) {
            angle += 360;
        }
    }

    void right(double currentAngle) {
        init(currentAngle);
        
        time = millis();
        pidI = 0;
        throttle = MOTORS_ROTATION_PWM;
        i = 0;
        prevDiff = currentAngle - angle;
        state = ROTATE;
        
        angle += 90;

        if (angle > 180) {
            angle -= 360;
        }
    }

    struct MotorPID {
        DIRECTION dir;

        double kp, ki, kd;
        double p = 0, i = 0, d = 0, diff = 0, prevDiff = 0, speed, throttle;
        
        MotorPID(double kp, double ki, double kd, double speed, double throttle) : kp(kp), ki(ki), kd(kd) {
            this->speed = speed;
            this->throttle = throttle;
        }
        
        double computePWM(double spd, double elapsedTime) {
            diff = speed - spd;

            p = diff * kp;
            i += diff * ki;
            d = (diff - prevDiff) * kd;

            prevDiff = diff;

            double pid = constrain(p + i + d, -PWMRANGE, PWMRANGE);

            throttle = constrain(throttle + pid, 0, PWMRANGE);

            return throttle;
        }

        double computeRPM(double ticks, double elapsedTime) {
            int sign = (dir == FWARD) ? 1 : -1;

            return sign * ((ticks / DISK_SLOTS) * float(60) / elapsedTime);
        }

        double computeDistance(double ticks) {
            int sign = (dir == FWARD) ? 1 : -1;

            return sign * ((ticks / DISK_SLOTS) * WHEEL_DIAMETER * M_PI);
        }

        void adjustSpeed(double spd) {
            speed = constrain(spd, 0, MOTORS_MAX_SPEED);
        }

        void reset() {
            p = i = d = diff = prevDiff = 0;
        }
    };

private:
    int i = 0;

    STATE state = INIT;

    Communicator* com;
    PCF857x* pcf1;
    
    Encoder* len;
    Encoder* ren;

    MotorPID leftMotorPID;
    MotorPID rightMotorPID;

    double angle = 0; // Reference angle
    double distance = 0; // Remaining distance

    double time = 0;
    double prevDiff = 0;
    double pid = 0, pidP = 0, pidI = 0, pidD = 0, throttle = MOTORS_ROTATION_PWM;

    double preAlignTime;

    bool atJoint = true, atEdge = false, atBlack = true;
    double atBlackTime = millis();

    void move(double currentAngle, unsigned long obstacleDistance, bool isLeftBlack, bool isRightBlack) {
        com->send(" ");

        // Emergency braking
        if (obstacleDistance < MIN_DISTANCE) {
            com->send("Error: nearby object!");

            adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);

            time = millis(); // To avoid large elapsed time after the obstacle is removed

            return;
        }

        // Reached a black spot
        // if (isBlack && millis() - atBlackTime > 250 && !atBlack) {
        //     if (atJoint && !atEdge) {
        //         atJoint = false;
        //         atEdge = true;

        //         angle = currentAngle; // Recalibrate angle here
        //     } else if (!atJoint && atEdge) {
        //         atJoint = true;
        //         atEdge = false;

        //         com->send("Distance: " + String(distance) + " isBlack: " + String(isBlack));
                
        //         adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);        

        //         state = ALIGN;
        //         preAlignTime = millis();
        //     }

        //     atBlackTime = millis();
        //     atBlack = true;
        // }

        // if (!isBlack) {
        //     atBlack = false;
        // }

        if (state == ALIGN) {
            return;
        }

        //
        // Timer
        //
        double current = millis();

        if (current - time < MOTORS_ADJUST_DELTA)
            return;

        double elapsedTime = (current - time) / 1000.0;
        time = current;

        //
        // Angle
        //
        double diff = currentAngle - angle;

        if (diff > 180) {
            diff -= 360;
        } else if (diff < -180) {
            diff += 360;
        }

        pidP = KP * diff;
        pidI += (diff < I_LIMIT && diff > -I_LIMIT) ? (KI * diff) : 0;
        pidD = KD * ((diff - prevDiff) / elapsedTime);
        prevDiff = diff;

        pid = constrain(pidP + pidI + pidD, -MOTORS_MAX_SPEED, MOTORS_MAX_SPEED);

        //
        // Distance
        //
        unsigned long leftTicks, rightTicks;
        
        noInterrupts();
        len->readAndReset(leftTicks);
        ren->readAndReset(rightTicks);
        interrupts();

        // Get motors distances
        double leftDistance = leftMotorPID.computeDistance(leftTicks);
        double rightDistance = rightMotorPID.computeDistance(rightTicks);
    
        // Get motors speeds
        double leftRPM = leftMotorPID.computeRPM(leftTicks, elapsedTime);
        double rightRPM = rightMotorPID.computeRPM(rightTicks, elapsedTime);

        // Adjust speed to maintain angle and reduce speed difference
        leftMotorPID.adjustSpeed(MOTORS_INIT_SPEED - pid - (leftRPM - rightRPM) / 2.);
        rightMotorPID.adjustSpeed(MOTORS_INIT_SPEED + pid + (leftRPM - rightRPM) / 2.);

        // Get motors throttle
        double leftPWM = leftMotorPID.computePWM(leftRPM, elapsedTime);
        double rightPWM = rightMotorPID.computePWM(rightRPM, elapsedTime);

        // Update distance
        distance -= (leftDistance + rightDistance) / 2;
        
        // if (isBlack) {
        //     com->send("Distance: " + String(distance) + " isBlack: " + String(isBlack));
            
        //     state = IDLE;
            
        //     adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);        

        //     return;
        // }

        com->send("MOVE:: elapsedTime: " + String(elapsedTime) + 
                  " - PID: " + String(pid) + 
                  " - DIFF: " + String(diff) + 
                  " - Angle: " + String(angle) + 
                  " - currentAngle: " + String(currentAngle) + 
                  " - LeftRPM: " + String(leftRPM) + 
                  " - LeftSpeed: " + String(leftMotorPID.speed) + 
                  " - LeftPWM: " + String(leftPWM) + 
                  " - LeftP: " + String(leftMotorPID.p) +
                  " - LeftI: " + String(leftMotorPID.i) +
                  " - LeftD: " + String(leftMotorPID.d) + 
                  " - RightRPM: " + String(rightRPM) +
                  " - RightSpeed: " + String(rightMotorPID.speed) + 
                  " - RightPWM: " + String(rightPWM) +
                  " - RightP: " + String(rightMotorPID.p) +
                  " - RightI: " + String(rightMotorPID.i) +
                  " - RightD: " + String(rightMotorPID.d) + 
                  " - leftDistance: " + String(leftDistance) + 
                  " - rightDistance: " + String(rightDistance) + 
                  " - Distance: " + String(distance));

        // Set directions
        int lDir1 = HIGH;
        int lDir2 = LOW;
        int rDir1 = HIGH;
        int rDir2 = LOW;
        
        if (leftPWM < 0 || leftMotorPID.speed < 0) {
            leftPWM = -leftPWM;

            Utils::swap(&lDir1, &lDir2);
        }
        
        if (rightPWM < 0 || rightMotorPID.speed < 0) {
            rightPWM = -rightPWM;

            Utils::swap(&rDir1, &rDir2);
        }

        adjustMotors(leftPWM, rightPWM, lDir1, lDir2, rDir1, rDir2);
    }

    void rotate(double currentAngle) {
        com->send(" ");
        
        //
        // Timer
        //
        double current = millis();

        if (current - time < MOTORS_ADJUST_DELTA * 2)
            return;

        double elapsedTime = (current - time) / 1000.0;
        time = current;

        //
        // Angle
        //
        double diff = currentAngle - angle;

        if (diff > 180) {
            diff -= 360;
        } else if (diff < -180) {
            diff += 360;
        }

        if (fabs(diff) <= 65) {
            com->send("Diff:: " + String(i++) + " " + String(diff));

            throttle = MOTORS_ROTATION_PWM;

            adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);        

            state = ALIGN;
            preAlignTime = millis();

            return;
        }

        //
        // PID
        //
        // if (fabs(diff) > 60) { 
        //     pidP = KP2 * diff;
        //     pidI = 0;
        //     pid = constrain(pidP + pidI, -PWMRANGE, PWMRANGE);
            
        //     if (diff >= 0) { // Turning left
        //         leftPWM = -MOTORS_ROTATION_PWM - pid;
        //         rightPWM = MOTORS_ROTATION_PWM + pid;
        //     } else {
        //         leftPWM = MOTORS_ROTATION_PWM - pid;
        //         rightPWM = -MOTORS_ROTATION_PWM + pid;
        //     }
        // } else {
        //     pidP = 2 * diff;
        //     pidI += 0.05 * diff;
        //     pid = constrain(pidP + pidI, -PWMRANGE, PWMRANGE);
            
        //     if (diff >= 0) { // Turning left
        //         leftPWM = -pid;
        //         rightPWM = pid;
        //     } else {
        //         leftPWM = -pid;
        //         rightPWM = pid;
        //     }
        // }
        
        double leftPWM, rightPWM;

        // if (fabs(diff) < fabs(prevDiff) && abs(diff) < 60) {
        //     throttle /= 1.2;
        // } else {
        
        if (abs(diff) >= abs(prevDiff)) {
            throttle += 5;
        } else if (fabs(prevDiff) - fabs(diff) >= 2) {
            // throttle /= 2;
        }

        // }

        prevDiff = diff;

        if (diff >= 0) { // Turning left
            leftPWM = -throttle;
            rightPWM = throttle;
        } else {
            leftPWM = throttle;
            rightPWM = -throttle;
        }

        com->send("ROTATE:: " + String(i++) + 
                  " - elapsedTime: " + String(elapsedTime) + 
                  " - PID: " + String(pid) + 
                  " - P: " + String(pidP) + 
                  " - I: " + String(pidI) + 
                  " - D: " + String(pidD) + 
                  " - DIFF: " + String(diff) + 
                  " - Angle: " + String(angle) + 
                  " - currentAngle: " + String(currentAngle) + 
                  " - LeftPWM: " + String(leftPWM) + 
                  " - RightPWM: " + String(rightPWM));

        // Set directions
        int lDir1 = HIGH;
        int lDir2 = LOW;
        int rDir1 = HIGH;
        int rDir2 = LOW;
        
        if (leftPWM < 0) {
            leftPWM = -leftPWM;

            Utils::swap(&lDir1, &lDir2);
        }
        
        if (rightPWM < 0) {
            rightPWM = -rightPWM;

            Utils::swap(&rDir1, &rDir2);
        }

        adjustMotors(leftPWM, rightPWM, lDir1, lDir2, rDir1, rDir2);
    }

    void align(double currentAngle) {
        com->send(" ");

        if (millis() - preAlignTime < PRE_ALIGN_PERIOD) { // Wait for the car to stop first
            return;
        }

        double current = millis();

        if (current - time < MOTORS_ADJUST_DELTA)
            return;

        double elapsedTime = (current - time) / 1000.0;
        time = current;

        double diff = currentAngle - angle;

        if (diff > 180) {
            diff -= 360;
        } else if (diff < -180) {
            diff += 360;
        }

        if (fabs(diff) <= 10) {
            // com->send("Diff:: " + String(i++) + " " + String(diff));

            adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);        

            return;
        } else if (fabs(diff) > 65) {
            state = ROTATE;

            return;
        }

        //
        // PID
        //
        double leftPWM, rightPWM;
        
        pidP = KP2 * diff;
        pidI += KI2 * diff;
        pid = constrain(pidP + pidI, -PWMRANGE, PWMRANGE);
        
        if (diff >= 0) { // Turning left
            leftPWM = -MOTORS_ALIGNMENT_PWM - pid;
            rightPWM = MOTORS_ALIGNMENT_PWM + pid;
        } else {
            leftPWM = MOTORS_ALIGNMENT_PWM - pid;
            rightPWM = -MOTORS_ALIGNMENT_PWM + pid;
        }

        com->send("ALIGN:: " + String(i++) + 
                  " - PID: " + String(pid) + 
                  " - P: " + String(pidP) + 
                  " - I: " + String(pidI) + 
                  " - D: " + String(pidD) + 
                  " - DIFF: " + String(diff) + 
                  " - Angle: " + String(angle) + 
                  " - currentAngle: " + String(currentAngle) + 
                  " - LeftPWM: " + String(leftPWM) + 
                  " - RightPWM: " + String(rightPWM));

        // Set directions
        int lDir1 = HIGH;
        int lDir2 = LOW;
        int rDir1 = HIGH;
        int rDir2 = LOW;
        
        if (leftPWM < 0) {
            leftPWM = -leftPWM;

            Utils::swap(&lDir1, &lDir2);
        }
        
        if (rightPWM < 0) {
            rightPWM = -rightPWM;

            Utils::swap(&rDir1, &rDir2);
        }

        adjustMotors(leftPWM, rightPWM, lDir1, lDir2, rDir1, rDir2);
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