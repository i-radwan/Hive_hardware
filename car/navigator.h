#pragma once

#include "communicator.h"
#include "sensors/encoder.h"
#include "utils/utils.h"
#include "utils/constants.h"

class Navigator {
public:

    // 1.2, 0.1, 0.4
    Navigator() : leftMotorPID(2.3, 0.3, 0, MOTORS_INIT_SPEED, 0), 
                  rightMotorPID(2.3, 0.3, 0, MOTORS_INIT_SPEED, 0) {
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

    bool navigate(double currentAngle, unsigned long distance, bool isFrontLeftBlack, bool isFrontRightBlack, bool isBackLeftBlack, bool isBackRightBlack, String& logs) {
        if (state == STRAIGHT || state == STRAIGHT_LEFT || state == STRAIGHT_RIGHT || state == OFFLINE_LEFT || state ==  OFFLINE_RIGHT || state == ALIGNMENT)
            return move(currentAngle, distance, isFrontLeftBlack, isFrontRightBlack, isBackLeftBlack, isBackRightBlack, logs);
        else if (state == ROTATE_LEFT || state == ROTATE_RIGHT || state == PRE_ROTATE_RIGHT || state == PRE_ROTATE_LEFT || state == POST_ROTATE_RIGHT || state == POST_ROTATE_LEFT)
            return rotate(currentAngle, isFrontLeftBlack, isFrontRightBlack, isBackLeftBlack, isBackRightBlack, logs);
        
        return false;
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

    void left(double currentAngle, bool isFrontLeftBlack, bool isFrontRightBlack) {
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

        // if (isFrontLeftBlack) {
            state = PRE_ROTATE_LEFT;
        // } else {
            // state = ROTATE_LEFT;
        // }

        wasBackRightBlack = false;
        wasBackLeftBlack = false;
        
        angle += -90;

        if (angle < -180) {
            angle += 360;
        }
    }

    void right(double currentAngle, bool isFrontLeftBlack, bool isBackRightBlack) {
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

        // if (isFrontLeftBlack) {
            state = PRE_ROTATE_RIGHT;
        // } else {
            // state = ROTATE_RIGHT;
        // }
        
        wasBackRightBlack = false;
        wasBackLeftBlack = false;

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

    double time = 0;
    double prevDiff = 0;
    double pid = 0, pidP = 0, pidI = 0, pidD = 0;

    bool atNode = true;
    double atBlackTime = millis();

    int spdDiffI = 0;

    bool prevIsFrontRightBlack = false;
    bool prevIsFrontLeftBlack = false;
    bool prevIsBackRightBlack = false;
    bool prevIsBackLeftBlack = false;

    bool wasBackLeftBlack = false;
    bool wasBackRightBlack = false;

    bool move(double currentAngle, unsigned long obstacleDistance, bool isFrontLeftBlack, bool isFrontRightBlack, bool isBackLeftBlack, bool isBackRightBlack, String& logs) {
        // com->send(" ");

        // Emergency braking
        // if (obstacleDistance < MIN_DISTANCE) {
        //     com->send("Error: nearby object!");

        //     adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);

        //     time = millis(); // To avoid large elapsed time after the obstacle is removed

        //     return;
        // }

        // Reached a black spot
        // if (isFrontLeftBlack && isFrontRightBlack && millis() - atBlackTime > 250 && !atBlack) {
        //     com->send("New Black Spot!");
            
        //     adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);        

        //     state = ALIGN;

        //     atBlackTime = millis();
        //     atBlack = true;
        // }

        // if (!isFrontLeftBlack || !isFrontRightBlack) {
        //     atBlack = false;
        // }

        // if (state == ALIGN) {
        //     return;
        // }

        //
        // Timer
        //
        double current = millis();

        if (current - time < MOTORS_ADJUST_DELTA && 
            prevIsFrontRightBlack == isFrontRightBlack && 
            prevIsFrontLeftBlack == isFrontLeftBlack && 
            prevIsBackRightBlack == isBackRightBlack && 
            prevIsBackLeftBlack == isBackLeftBlack)
            return false;

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
        prevIsFrontRightBlack = isFrontRightBlack;
        prevIsFrontLeftBlack = isFrontLeftBlack;
        prevIsBackRightBlack = isBackRightBlack;
        prevIsBackLeftBlack = isBackLeftBlack;

        // FSM transitions
        switch (state) {
            case STRAIGHT:
                if (isFrontLeftBlack && !isFrontRightBlack) {
                    state = STRAIGHT_RIGHT;
                    spdDiffI = 0;
                } else if (!isFrontLeftBlack && isFrontRightBlack) {
                    state = STRAIGHT_LEFT;
                    spdDiffI = 0;
                } 

                if (!isBackLeftBlack && !isBackRightBlack) {
                    atNode = false;
                }
            break;

            case STRAIGHT_LEFT:
                if (!isFrontRightBlack /*&& diff * entryDiff < 0*/) { // Signs changed
                    state = STRAIGHT;
                } 
                // else if (!isFrontRightBlack && diff * entryDiff > 0) { // Same sign
                //     state = OFFLINE_LEFT;
                //     spdDiffI = 0;
                // } 

                if (!isBackLeftBlack && !isBackRightBlack) {
                    atNode = false;
                }
            break;

            case STRAIGHT_RIGHT:
                if (!isFrontLeftBlack /*&& diff * entryDiff < 0*/) { // Signs changed
                    state = STRAIGHT;
                } 
                // else if (!isFrontLeftBlack && diff * entryDiff > 0) { // Same sign
                //     state = OFFLINE_RIGHT;
                //     spdDiffI = 0;
                // }

                if (!isBackLeftBlack && !isBackRightBlack) {
                    atNode = false;
                }
            break;

            // case OFFLINE_LEFT:
            //     if (isFrontRightBlack) {
            //         state = STRAIGHT;
            //     }
            // break;

            // case OFFLINE_RIGHT:
            //     if (isFrontLeftBlack) {
            //         state = STRAIGHT;
            //     }
            break;
        }

        double spdDiff = 0;
        double leftSpeed, rightSpeed;

        // States actions
        switch (state) {
            case STRAIGHT:
                leftSpeed = MOTORS_INIT_SPEED;
                rightSpeed = MOTORS_INIT_SPEED;
                // spdDiff = pid, spdDiffI = 0;
            break;

            case STRAIGHT_LEFT:
                leftSpeed = MOTORS_INIT_SPEED;
                rightSpeed = MOTORS_INIT_SPEED / 2.5;
                // spdDiffI -= 2, spdDiff = -20 * fabs(diff / angle);

                // if (abs(diff) >= 5) { // Stop the wheel to lock the car to the line
                //     rightFactor = 0;
                //     leftFactor = 0.75;
                // }
            break;

            case STRAIGHT_RIGHT:
                leftSpeed = MOTORS_INIT_SPEED / 2.5;
                rightSpeed = MOTORS_INIT_SPEED;
                
                // spdDiffI -= 2, spdDiff = 20 * fabs(diff / angle);

                // if (abs(diff) >= 5) { // Stop the wheel to lock the car to the line
                //     rightFactor = 0.75;
                //     leftFactor = 0;
                // }
            break;

            // case OFFLINE_LEFT:
            //     spdDiffI -= 4, spdDiff = -40 - spdDiffI;
            // break;

            // case OFFLINE_RIGHT:
            //     spdDiffI -= 4, spdDiff = 40 + spdDiffI;
            // break;
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
        // leftMotorPID.adjustSpeed((MOTORS_INIT_SPEED - spdDiff) * leftFactor);
        // rightMotorPID.adjustSpeed((MOTORS_INIT_SPEED + spdDiff) * rightFactor);
        leftMotorPID.adjustSpeed(leftSpeed);
        rightMotorPID.adjustSpeed(rightSpeed);

        // Get motors throttle
        double leftPWM = leftMotorPID.computePWM(leftRPM, elapsedTime);
        double rightPWM = rightMotorPID.computePWM(rightRPM, elapsedTime);

        // Stopping at nodes
        if (isBackLeftBlack && !atNode) {
            leftPWM = 0;
        }

        if (isBackRightBlack && !atNode) {
            rightPWM = 0;
        }

        if (isBackLeftBlack && isBackRightBlack && !atNode) {
            adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);        

            state = IDLE;
            atNode = true;

            logs += (" - Stopping");

            return true;
        }

        logs += ("MOVE:: elapsedTime: " + String(elapsedTime) + 
                " - STATE: " + String(state) + 
                " - atNode: " + String(atNode) + 
                  // " - PID: " + String(pid) + 
                  // " - P: " + String(pidP) + 
                  // " - I: " + String(pidI) + 
                  // " - D: " + String(pidD) + 
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
                  // " - leftDistance: " + String(leftDistance) + 
                  // " - rightDistance: " + String(rightDistance) + 
                  " - Distance: " + String(distance) + 
                  " - isFrontLeftBlack: " + String(isFrontLeftBlack) +
                  " - isFrontRightBlack: " + String(isFrontRightBlack) +
                  " - isBackLeftBlack: " + String(isBackLeftBlack) +
                  " - isBackRightBlack: " + String(isBackRightBlack) +
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

        // adjustMotors(leftSpeed == 0 ? 0 : leftPWM, rightSpeed == 0 ? 0 : rightPWM, lDir1, lDir2, rDir1, rDir2);
        adjustMotors(leftPWM, rightPWM, lDir1, lDir2, rDir1, rDir2);
    }

    bool rotate(double currentAngle, bool isFrontLeftBlack, bool isFrontRightBlack, bool isBackLeftBlack, bool isBackRightBlack, String& logs) {
        //
        // Timer
        //
        double current = millis();
        
        if (current - time < MOTORS_ADJUST_DELTA && 
            prevIsFrontRightBlack == isFrontRightBlack && 
            prevIsFrontLeftBlack == isFrontLeftBlack && 
            prevIsBackRightBlack == isBackRightBlack && 
            prevIsBackLeftBlack == isBackLeftBlack)
            return false;

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
        // Line following
        //
        prevIsFrontRightBlack = isFrontRightBlack;
        prevIsFrontLeftBlack = isFrontLeftBlack;
        prevIsBackRightBlack = isBackRightBlack;
        prevIsBackLeftBlack = isBackLeftBlack;

        // FSM transitions
        switch (state) {
            // case PRE_ROTATE_RIGHT:
            //     if (!isFrontRightBlack) {
            //         state = ROTATE_RIGHT;
            //     }
            // break;
            // case PRE_ROTATE_LEFT:
            //     if (!isFrontLeftBlack) {
            //         state = ROTATE_LEFT;
            //     }
            // break;
            
            // case ROTATE_RIGHT:
            //     if (isFrontRightBlack) {
            //         state = POST_ROTATE_RIGHT;
            //     }
            // break;
            
            // case ROTATE_LEFT: 
            //     if (isFrontLeftBlack) {
            //         state = POST_ROTATE_LEFT;
            //     }
            // break;
            
            // case POST_ROTATE_RIGHT: 
            //     if (!isFrontRightBlack) {
            //         state = IDLE;
            //     }
            // break;
            
            // case POST_ROTATE_LEFT: 
            //     if (!isFrontLeftBlack) {
            //         state = IDLE;
            //     }
            // break;
            case PRE_ROTATE_RIGHT:
                if (!isBackRightBlack && !isBackLeftBlack && !isFrontRightBlack) {
                    state = ROTATE_RIGHT;
                }
            break;
            case PRE_ROTATE_LEFT:
                if (!isBackRightBlack && !isBackLeftBlack && !isFrontLeftBlack) {
                    state = ROTATE_LEFT;
                }
            break;
            
            case ROTATE_RIGHT:
                if (isFrontRightBlack) {
                    state = POST_ROTATE_RIGHT;
                }
            break;
            
            case ROTATE_LEFT: 
                if (isFrontLeftBlack) {
                    state = POST_ROTATE_LEFT;
                }
            break;

            case POST_ROTATE_RIGHT:
                if (wasBackLeftBlack && wasBackRightBlack) {
                    state = IDLE;
                }
            break;

            case POST_ROTATE_LEFT:
                if (wasBackLeftBlack && wasBackRightBlack) {
                    state = IDLE;
                }
            break;
        }

        double lspd = 0;
        double rspd = 0;

        int lFactor = 1, rFactor = 1;

        // States actions
        switch (state) {
            case IDLE:
                adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);        
            
                noInterrupts();
                len->reset();
                ren->reset();
                interrupts();

                return true;
            break;

            case PRE_ROTATE_RIGHT:
                rspd = -MOTORS_ROTATION_SPEED;
                lspd = MOTORS_ROTATION_SPEED;
            break;

            case PRE_ROTATE_LEFT:
                rspd = MOTORS_ROTATION_SPEED;
                lspd = -MOTORS_ROTATION_SPEED;
            break;
            
            case ROTATE_RIGHT:
                rspd = -MOTORS_ROTATION_SPEED;
                lspd = MOTORS_ROTATION_SPEED;
            break;
            
            case ROTATE_LEFT:
                rspd = MOTORS_ROTATION_SPEED;
                lspd = -MOTORS_ROTATION_SPEED;
            break;
            
            case POST_ROTATE_RIGHT:
                rspd = -MOTORS_ROTATION_SPEED;
                lspd = MOTORS_ROTATION_SPEED;

                if (isBackLeftBlack) {
                    wasBackLeftBlack = true;
                }
                
                if (isBackRightBlack) {
                    wasBackRightBlack = true;
                }

                if (wasBackRightBlack) {
                    rspd = 0;
                    rFactor = 0;
                }

                if (wasBackLeftBlack) {
                    lspd = 0;
                    lFactor = 0;
                }
            break;
            
            case POST_ROTATE_LEFT:
                rspd = MOTORS_ROTATION_SPEED;
                lspd = -MOTORS_ROTATION_SPEED;

                if (isBackLeftBlack) {
                    wasBackLeftBlack = true;
                }
                
                if (isBackRightBlack) {
                    wasBackRightBlack = true;
                }

                if (wasBackRightBlack) {
                    rspd = 0;
                    rFactor = 0;
                }

                if (wasBackLeftBlack) {
                    lspd = 0;
                    lFactor = 0;
                }
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
                  " - isFrontLeftBlack: " + String(isFrontLeftBlack) +
                  " - isFrontRightBlack: " + String(isFrontRightBlack) +
                  " - isBackLeftBlack: " + String(isBackLeftBlack) +
                  " - isBackRightBlack: " + String(isBackRightBlack) +
                  " - wasBackLeftBlack: " + String(wasBackLeftBlack) +
                  " - wasBackRightBlack: " + String(wasBackRightBlack) +
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

        adjustMotors(leftPWM * lFactor, rightPWM * rFactor, lDir1, lDir2, rDir1, rDir2);

        return false;
    }

    void adjustMotors(int lSpeed, int rSpeed, int lDir1, int lDir2, int rDir1, int rDir2) {
        // Lock the stopping wheels
        if (lSpeed < 1e-6) {
            lDir1 = lDir2 = HIGH;
            lSpeed = PWMRANGE;
        }

        if (rSpeed < 1e-6) {
            rDir1 = rDir2 = HIGH;
            rSpeed = PWMRANGE;
        }

        analogWrite(LEFT_SPED, lSpeed);
        analogWrite(RGHT_SPED, rSpeed);

        pcf1->write(LEFT_DIR1, lDir1);
        pcf1->write(LEFT_DIR2, lDir2);
        pcf1->write(RGHT_DIR1, rDir1);
        pcf1->write(RGHT_DIR2, rDir2);
    }
};