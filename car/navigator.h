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

    void navigate(double currentAngle) {
        if (state == MOVE)
            move(currentAngle);
        else if (state == ALIGN || state == ROTATE)
            rotate(currentAngle);
    }
    
    void stop(double currentAngle) {
        if (state == INIT) 
            return;
        else if (state == IDLE) // ToDo: remove
            init(currentAngle);

        state = IDLE;

        // ToDo: reconsider later
        noInterrupts();
        len->reset();
        ren->reset();
        interrupts();

        adjustMotors(PWMRANGE, PWMRANGE, LOW, LOW, LOW, LOW);
    }
    
    void forward(double currentAngle) {
        init(currentAngle);

        time = millis();
        pidI = 0;
        state = MOVE;

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

            double pid = p + i + d;
            pid = constrain(pid, -PWMRANGE, PWMRANGE);

            throttle = constrain(throttle + pid, -PWMRANGE, PWMRANGE);

            dir = (throttle >= 0) ? FWARD : BWARD;

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
            speed = constrain(spd, -MOTORS_MAX_SPEED, MOTORS_MAX_SPEED);
        }

        void reset() {
            p = i = d = diff = prevDiff = 0;
        }
    };

private:
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
    double pid = 0, pidP = 0, pidI = 0, pidD = 0;

    void move(double currentAngle) {
        com->send(" ");
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

        leftMotorPID.adjustSpeed(MOTORS_INIT_SPEED - pid);
        rightMotorPID.adjustSpeed(MOTORS_INIT_SPEED + pid);

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

        // Get motors throttle
        double leftPWM = leftMotorPID.computePWM(leftRPM, elapsedTime);
        double rightPWM = rightMotorPID.computePWM(rightRPM, elapsedTime);

        // Update distance
        distance -= (leftDistance + rightDistance) / 2;
        
        if (distance <= 0) {
            com->send("Distance: " + String(distance));
            
            state = ALIGN;

            return;
        }

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

    void rotate(double currentAngle) {
        //
        // Timer
        //
        double current = millis();
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

        if (abs(diff) <= 3) {
            // com->send("Diff: " + String(diff));

            adjustMotors(PWMRANGE, PWMRANGE, LOW, LOW, LOW, LOW);        
            
            noInterrupts();
            len->reset();
            ren->reset();
            interrupts();

            return;
        }

        //
        // PID
        //
        pidP = KP2 * diff;
        pidI += KI2 * diff;
        pidD = KD2 * ((diff - prevDiff) / elapsedTime);
        prevDiff = diff;

        pid = constrain(pidP + pidI + pidD, -PWMRANGE, PWMRANGE);

        double leftPWM, rightPWM;
        
        if (diff >= 0) { // Turning left
            leftPWM = -MOTORS_ROTATION_PWM - pid;
            rightPWM = MOTORS_ROTATION_PWM + pid;
        } else {
            leftPWM = MOTORS_ROTATION_PWM - pid;
            rightPWM = -MOTORS_ROTATION_PWM + pid;
        }

        com->send("ROTATE:: elapsedTime: " + String(elapsedTime) + 
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