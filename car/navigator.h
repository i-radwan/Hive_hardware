#pragma once

#include "communicator.h"
#include "sensors/encoder.h"
#include "utils/utils.h"
#include "utils/constants.h"

class Navigator {
public:

    // 1.2, 0.1, 0.4
    Navigator() : leftMotorPID(1.3, 0.1, 0, MOTORS_INIT_SPEED, 0), 
                  rightMotorPID(1.3, 0.1, 0, MOTORS_INIT_SPEED, 0) {
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

    void navigate(double currentAngle, unsigned long distance, bool isLeftBlack, bool isRightBlack, String& logs) {
        if (state == STRAIGHT || state == STRAIGHT_LEFT || state == STRAIGHT_RIGHT || state == OFFLINE_LEFT || state ==  OFFLINE_RIGHT || state == ALIGNMENT)
            move(currentAngle, distance, isLeftBlack, isRightBlack, logs);
        else if (state == ROTATE_LEFT || state == ROTATE_RIGHT || state == PRE_ROTATE_RIGHT || state == PRE_ROTATE_LEFT || state == POST_ROTATE_RIGHT || state == POST_ROTATE_LEFT)
            rotate(currentAngle, isLeftBlack, isRightBlack, logs);
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
        prevDiff = currentAngle - angle;

        leftMotorPID.reset();
        rightMotorPID.reset();

        noInterrupts();
        len->reset();
        ren->reset();
        interrupts();

        state = STRAIGHT;
    }

    void backward(double currentAngle) {
        // init(currentAngle);
        
        // time = millis();
        // pidI = 0;
    }

    void left(double currentAngle, bool isLeftBlack, bool isRightBlack) {
        init(currentAngle);
        
        time = millis();
        pidI = 0;
        i = 0;
        prevDiff = currentAngle - angle;

        leftMotorPID.reset();
        rightMotorPID.reset();

        noInterrupts();
        len->reset();
        ren->reset();
        interrupts();

        if (isLeftBlack) {
            state = PRE_ROTATE_LEFT;
        } else {
            state = ROTATE_LEFT;
        }
        
        angle += -90;

        if (angle < -180) {
            angle += 360;
        }
    }

    void right(double currentAngle, bool isLeftBlack, bool isRightBlack) {
        init(currentAngle);
        
        time = millis();
        pidI = 0;
        i = 0;
        prevDiff = currentAngle - angle;

        leftMotorPID.reset();
        rightMotorPID.reset();

        noInterrupts();
        len->reset();
        ren->reset();
        interrupts();

        if (isRightBlack) {
            state = PRE_ROTATE_RIGHT;
        } else {
            state = ROTATE_RIGHT;
        }
        
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
            d = (elapsedTime > 1e-6) ? ((diff - prevDiff) / elapsedTime * kd) : 0;

            prevDiff = diff;

            double pid = constrain(p + i + d, -PWMRANGE, PWMRANGE);

            throttle = constrain(PWMRANGE / 5. + pid, 0, PWMRANGE);

            return throttle;
        }

        double computeRPM(double ticks, double elapsedTime) {
            return (ticks / DISK_SLOTS) * float(60) / elapsedTime;
        }

        double computeDistance(double ticks) {
            return (ticks / DISK_SLOTS) * WHEEL_DIAMETER * M_PI;
        }

        void adjustSpeed(double spd) {
            speed = constrain(spd, 0, MOTORS_MAX_SPEED);
        }

        void reset() {
            p = i = d = diff = prevDiff = throttle = 0;
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

    bool metered = false;

    double time = 0;
    double prevDiff = 0;
    double pid = 0, pidP = 0, pidI = 0, pidD = 0;

    bool atNode = false; // ToDo: make true in complete grid
    double atBlackTime = millis();

    int spdDiffI = 0;

    bool prevIsRightBlack = false;
    bool prevIsLeftBlack = false;
    double entryDiff = 0;

    void move(double currentAngle, unsigned long obstacleDistance, bool isLeftBlack, bool isRightBlack, String& logs) {
        // com->send(" ");

        // Emergency braking
        // if (obstacleDistance < MIN_DISTANCE) {
        //     com->send("Error: nearby object!");

        //     adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);

        //     time = millis(); // To avoid large elapsed time after the obstacle is removed

        //     return;
        // }

        // Reached a black spot
        // if (isLeftBlack && isRightBlack && millis() - atBlackTime > 250 && !atBlack) {
        //     com->send("New Black Spot!");
            
        //     adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);        

        //     state = ALIGN;

        //     atBlackTime = millis();
        //     atBlack = true;
        // }

        // if (!isLeftBlack || !isRightBlack) {
        //     atBlack = false;
        // }

        // if (state == ALIGN) {
        //     return;
        // }

        //
        // Timer
        //
        double current = millis();

        if (current - time < MOTORS_ADJUST_DELTA && prevIsRightBlack == isRightBlack && prevIsLeftBlack == isLeftBlack)
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
        pidD = (elapsedTime > 1e-6) ? (KD * ((diff - prevDiff) / elapsedTime)) : 0;
        prevDiff = diff;

        pid = constrain(pidP + pidI + pidD, -MOTORS_MAX_SPEED, MOTORS_MAX_SPEED);

        //
        // Line following
        //
        prevIsRightBlack = isRightBlack;
        prevIsLeftBlack = isLeftBlack;

        // FSM transitions
        switch (state) {
            case STRAIGHT:
                if (isLeftBlack && !isRightBlack) {
                    state = STRAIGHT_RIGHT;
                    entryDiff = diff;
                    spdDiffI = 0;
                } else if (!isLeftBlack && isRightBlack) {
                    state = STRAIGHT_LEFT;
                    entryDiff = diff;
                    spdDiffI = 0;
                } else {
                    atNode = false;
                }
            break;

            case STRAIGHT_LEFT:
                if (!isRightBlack /*&& diff * entryDiff < 0*/) { // Signs changed
                    state = STRAIGHT;
                } else if (!isRightBlack && diff * entryDiff > 0) { // Same sign
                    // state = OFFLINE_LEFT;
                    // spdDiffI = 0;
                }
            break;

            case STRAIGHT_RIGHT:
                if (!isLeftBlack /*&& diff * entryDiff < 0*/) { // Signs changed
                    state = STRAIGHT;
                } else if (!isLeftBlack && diff * entryDiff > 0) { // Same sign
                    // state = OFFLINE_RIGHT;
                    // spdDiffI = 0;
                }
            break;

            case OFFLINE_LEFT:
                if (isRightBlack) {
                    state = STRAIGHT;
                }
            break;

            case OFFLINE_RIGHT:
                if (isLeftBlack) {
                    state = STRAIGHT;
                }
            break;
        }

        double spdDiff = 0;
        double leftFactor = 1, rightFactor = 1;

        // States actions
        switch (state) {
            case STRAIGHT:
                spdDiff = pid, spdDiffI = 0;
            break;

            case STRAIGHT_LEFT:
                spdDiffI -= 2, spdDiff = -20 * fabs((diff / entryDiff));

                if (abs(diff) >= 5) { // Stop the wheel to lock the car to the line
                    rightFactor = 0;
                }
            break;

            case STRAIGHT_RIGHT:
                spdDiffI -= 2, spdDiff = 20 * fabs((diff / entryDiff));

                if (abs(diff) >= 5) { // Stop the wheel to lock the car to the line
                    leftFactor = 0;
                }
            break;

            case OFFLINE_LEFT:
                spdDiffI -= 4, spdDiff = -40 - spdDiffI;
            break;

            case OFFLINE_RIGHT:
                spdDiffI -= 4, spdDiff = 40 + spdDiffI;
            break;
        }

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
        leftMotorPID.adjustSpeed((MOTORS_INIT_SPEED - spdDiff) * leftFactor);
        rightMotorPID.adjustSpeed((MOTORS_INIT_SPEED + spdDiff) * rightFactor);

        // Get motors throttle
        double leftPWM = leftMotorPID.computePWM(leftRPM, elapsedTime);
        double rightPWM = rightMotorPID.computePWM(rightRPM, elapsedTime);

        // Update distance
        if (metered) {
            distance -= max(leftDistance, rightDistance);
                
            if (distance <= 0) {
                adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);        

                state = IDLE;
                metered = false;
                atNode = true;

                return;
            }            
        }

        // Set metered to move specific distance. Note we don't want to subtract the distance in this iteration
        // because this is the distance before we sensed the node
        if (isLeftBlack && isRightBlack && !atNode) {
            metered = true;
            distance += STEP;
        }

        logs += ("MOVE:: elapsedTime: " + String(elapsedTime) + 
                " - STATE: " + String(state) + 
                  // " - PID: " + String(pid) + 
                  // " - P: " + String(pidP) + 
                  // " - I: " + String(pidI) + 
                  // " - D: " + String(pidD) + 
                  " - DIFF: " + String(diff) + 
                  // " - Angle: " + String(angle) + 
                  // " - currentAngle: " + String(currentAngle) + 
                  " - LeftRPM: " + String(leftRPM) + 
                  " - LeftSpeed: " + String(leftMotorPID.speed) + 
                  " - LeftPWM: " + String(leftPWM) + 
                  // " - LeftP: " + String(leftMotorPID.p) +
                  // " - LeftI: " + String(leftMotorPID.i) +
                  // " - LeftD: " + String(leftMotorPID.d) + 
                  " - RightRPM: " + String(rightRPM) +
                  " - RightSpeed: " + String(rightMotorPID.speed) + 
                  " - RightPWM: " + String(rightPWM) +
                  // " - RightP: " + String(rightMotorPID.p) +
                  // " - RightI: " + String(rightMotorPID.i) +
                  // " - RightD: " + String(rightMotorPID.d) + 
                  // " - leftDistance: " + String(leftDistance) + 
                  // " - rightDistance: " + String(rightDistance) + 
                  " - Metered: " + String(metered) + 
                  " - Distance: " + String(distance) + 
                  " - isLeftBlack: " + String(isLeftBlack) +
                  " - isRightBlack: " + String(isRightBlack) +
                  " - spdDiff: " + String(spdDiff) +
                  " - spdDiffI: " + String(spdDiffI));

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

    void rotate(double currentAngle, bool isLeftBlack, bool isRightBlack, String& logs) {
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

        //
        // FSM transitions
        //
        switch (state) {
            case PRE_ROTATE_RIGHT:
                if (!isRightBlack) {
                    state = ROTATE_RIGHT;
                }
            break;
            case PRE_ROTATE_LEFT:
                if (!isLeftBlack) {
                    state = ROTATE_LEFT;
                }
            break;
            
            case ROTATE_RIGHT:
                if (isRightBlack) {
                    state = IDLE;
                }
            break;
            
            case ROTATE_LEFT: 
                if (isLeftBlack) {
                    state = IDLE;
                }
            break;
            
            case POST_ROTATE_RIGHT: 
                if (!isRightBlack) {
                    state = IDLE;
                }
            break;
            
            case POST_ROTATE_LEFT: 
                if (!isLeftBlack) {
                    state = IDLE;
                }
            break;
        }

        double lspd = 0;
        double rspd = 0;

        // States actions
        switch (state) {
            case IDLE:
                adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);        
            
                noInterrupts();
                len->reset();
                ren->reset();
                interrupts();

                return;
            break;

            case PRE_ROTATE_RIGHT:
                rspd = -30;
                lspd = 30;
            break;

            case PRE_ROTATE_LEFT:
                rspd = 30;
                lspd = -30;
            break;
            
            case ROTATE_RIGHT:
                rspd = -30;
                lspd = 30;
            break;
            
            case ROTATE_LEFT:
                rspd = 30;
                lspd = -30;
            break;
            
            case POST_ROTATE_RIGHT:
                rspd = -30;
                lspd = 30;
            break;
            
            case POST_ROTATE_LEFT:
                rspd = 30;
                lspd = -30;
            break;
        }

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
        leftMotorPID.adjustSpeed(abs(lspd));
        rightMotorPID.adjustSpeed(abs(rspd));

        // Get motors throttle
        double leftPWM = leftMotorPID.computePWM(leftRPM, elapsedTime);
        double rightPWM = rightMotorPID.computePWM(rightRPM, elapsedTime);

        logs += ("ROTATE:: " + String(i++) + 
                  " - state: " + String(state) + 
                  " - elapsedTime: " + String(elapsedTime) + 
                  // " - PID: " + String(pid) + 
                  // " - P: " + String(pidP) + 
                  // " - I: " + String(pidI) + 
                  // " - D: " + String(pidD) + 
                  // " - DIFF: " + String(diff) + 
                  // " - Angle: " + String(angle) + 
                  // " - currentAngle: " + String(currentAngle) + 
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
                  " - RightD: " + String(rightMotorPID.d));

        // Set directions
        int lDir1 = HIGH;
        int lDir2 = LOW;
        int rDir1 = HIGH;
        int rDir2 = LOW;
        
        if (lspd < 0) {
            Utils::swap(&lDir1, &lDir2);
        }
        
        if (rspd < 0) {
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