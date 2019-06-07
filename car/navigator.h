#pragma once

#include "communicator.h"
#include "sensors/encoder.h"
#include "utils/utils.h"
#include "utils/constants.h"

struct MotorController {
    
private: 

    const double kp, ki, kd;
    double p = 0, i = 0, d = 0, diff = 0, prevDiff = 0, speed;
    
    double computePWM(double spd, double elapsedTime) {
        diff = speed - spd;

        p = diff * kp;
        i += diff * ki;
        d = (elapsedTime > EPS) ? ((diff - prevDiff) / elapsedTime * kd) : 0;

        prevDiff = diff;

        double pid = constrain(p + i + d, -PWMRANGE, PWMRANGE);

        double throttle = constrain(PWMRANGE / 5. + pid, 0, PWMRANGE);

        return throttle;
    }

    double computeRPM(double ticks, double elapsedTime) {
        return (elapsedTime > EPS) ? ((ticks / DISK_SLOTS) * float(60) / elapsedTime) : 0;
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

public:

    Encoder* en;

    MotorController(double kp, double ki, double kd, double spd) : kp(kp), ki(ki), kd(kd) {
        speed = spd;
    }
    
    double compute(double elapsedTime, double newSpeed, double& distance) {
        adjustSpeed(newSpeed);

        unsigned long ticks = computeTicks();
        distance = computeDistance(ticks);
        
        double RPM = computeRPM(ticks, elapsedTime);
        double PWM = computePWM(RPM, elapsedTime);

        return PWM;
    }
    
    double compute(double elapsedTime, double newSpeed) {
        double distance;

        return compute(elapsedTime, newSpeed, distance);
    }

    void adjustSpeed(double spd) {
        speed = constrain(spd, 0, MOTORS_MAX_SPEED);
    }

    void reset() {
        p = i = d = diff = prevDiff = 0;

        noInterrupts();
        en->reset();
        interrupts();
    }
};

class Navigator {

public:

    // 1.2, 0.1, 0.4
    Navigator() : leftMotorController(2, 0.3, 0, MOTORS_INIT_SPEED), 
                  rightMotorController(2, 0.3, 0, MOTORS_INIT_SPEED) {
    }

    void setup(PCF857x* pcf1, Encoder* len, Encoder* ren) {
        // Motor pins
        pinMode(LEFT_SPED, OUTPUT);
        pinMode(RGHT_SPED, OUTPUT);

        leftMotorController.en = len;
        rightMotorController.en = ren;

        motorsPCF = pcf1;
    }

    bool navigate(unsigned long distance, bool isFrontCenterBlack, bool isFrontLeftBlack, bool isFrontRightBlack, bool isBackLeftBlack, bool isBackRightBlack, String& logs) {
        if (state == MOVE || state == STRAIGHT || state == STRAIGHT_LEFT || state == STRAIGHT_RIGHT || state == OFFLINE_LEFT || state ==  OFFLINE_RIGHT || state == ALIGNMENT)
            return move(distance, isFrontCenterBlack, isFrontLeftBlack, isFrontRightBlack, isBackLeftBlack, isBackRightBlack, logs);
        else if (state == ROTATE_LEFT || state == ROTATE_RIGHT || state == PRE_ROTATE_RIGHT || state == PRE_ROTATE_LEFT || state == PRE_ROTATE_RIGHT_2 || state == PRE_ROTATE_LEFT_2 || state == POST_ROTATE_RIGHT || state == POST_ROTATE_LEFT)
            return rotate(isFrontCenterBlack, isFrontLeftBlack, isFrontRightBlack, isBackLeftBlack, isBackRightBlack, logs);
        else if (state == PRE_RETREAT || state == RETREAT)
            return retreat(isFrontCenterBlack, isFrontLeftBlack, isFrontRightBlack, isBackLeftBlack, isBackRightBlack, logs);
        
        return false;
    }
    
    void stop() {
        state = IDLE;

        adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);
    }
    
    void prepare() {
        time = millis();

        leftMotorController.reset();
        rightMotorController.reset();

        wasBackRightBlack = false;
        wasBackLeftBlack = false;
    }

    void forward() {
        prepare();

        state = MOVE;

        distance = STEP;
    }

    void backward() {
        prepare();

        state = PRE_RETREAT;
    }

    void left() {
        prepare();

        state = PRE_ROTATE_LEFT;
    }

    void right() {
        prepare();

        state = PRE_ROTATE_RIGHT;
    }

private:
    STATE state = INIT;

    PCF857x* motorsPCF;
    
    MotorController leftMotorController;
    MotorController rightMotorController;

    double distance = 0; // Remaining distance
    double time = 0;

    // Navigation flags
    bool atNode = true;
    bool prevIsFrontRightBlack = false;
    bool prevIsFrontLeftBlack = false;
    bool prevIsBackRightBlack = false;
    bool prevIsBackLeftBlack = false;
    bool prevIsFrontCenterBlack = false;
    bool wasBackLeftBlack = false;
    bool wasBackRightBlack = false;

    bool move(unsigned long obstacleDistance, bool isFrontCenterBlack, bool isFrontLeftBlack, bool isFrontRightBlack, bool isBackLeftBlack, bool isBackRightBlack, String& logs) {
        // Emergency braking ToDo
        // if (obstacleDistance < MIN_DISTANCE) {
        //     log += ("Error: nearby object!\n");

        //     adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);

        //     time = millis(); // To avoid large elapsed time after the obstacle is removed

        //     return;
        // }

        //
        // Timer
        //
        double current = millis();

        if (current - time < MOTORS_ADJUST_DELTA && 
            prevIsFrontRightBlack == isFrontRightBlack && 
            prevIsFrontLeftBlack == isFrontLeftBlack && 
            prevIsFrontCenterBlack == isFrontCenterBlack && 
            prevIsBackRightBlack == isBackRightBlack && 
            prevIsBackLeftBlack == isBackLeftBlack)
            return false;

        double elapsedTime = (current - time) / 1000.0;
        time = current;

        //
        // Line following
        //
        prevIsFrontRightBlack = isFrontRightBlack;
        prevIsFrontLeftBlack = isFrontLeftBlack;
        prevIsFrontCenterBlack = isFrontCenterBlack;
        prevIsBackRightBlack = isBackRightBlack;
        prevIsBackLeftBlack = isBackLeftBlack;

        // FSM transitions
        switch (state) {
            case MOVE:
                if (isFrontLeftBlack && isFrontRightBlack && isFrontCenterBlack)
                    state = STRAIGHT;
                else if (!isFrontLeftBlack && !isFrontRightBlack && isFrontCenterBlack)
                    state = STRAIGHT;
                else if (!isFrontLeftBlack && isFrontRightBlack && isFrontCenterBlack)
                    state = STRAIGHT_LEFT;
                else if (isFrontLeftBlack && !isFrontRightBlack && isFrontCenterBlack)
                    state = STRAIGHT_RIGHT;
                else if (!isFrontLeftBlack && isFrontRightBlack && !isFrontCenterBlack)
                    state = OFFLINE_LEFT;
                else if (isFrontLeftBlack && !isFrontRightBlack && !isFrontCenterBlack)
                    state = OFFLINE_RIGHT;
            break;

            case STRAIGHT:
                if (isFrontLeftBlack && !isFrontRightBlack && isFrontCenterBlack) {
                    state = STRAIGHT_RIGHT;
                } else if (isFrontLeftBlack && !isFrontRightBlack && !isFrontCenterBlack) {
                    state = OFFLINE_RIGHT;
                } else if (!isFrontLeftBlack && isFrontRightBlack && isFrontCenterBlack) {
                    state = STRAIGHT_LEFT;
                } else if (!isFrontLeftBlack && isFrontRightBlack && !isFrontCenterBlack) {
                    state = OFFLINE_LEFT;
                } 

                if (!isBackLeftBlack && !isBackRightBlack) {
                    atNode = false;
                }
            break;

            case STRAIGHT_LEFT:
                if (!isFrontRightBlack && isFrontCenterBlack) {
                    state = STRAIGHT;
                } else if (!isFrontCenterBlack) {
                    state = OFFLINE_LEFT;
                } else if (isFrontLeftBlack && isFrontRightBlack) {
                    state = STRAIGHT;
                }

                if (!isBackLeftBlack && !isBackRightBlack) {
                    atNode = false;
                }
            break;

            case STRAIGHT_RIGHT:
                if (!isFrontLeftBlack && isFrontCenterBlack) {
                    state = STRAIGHT;
                } else if (!isFrontCenterBlack) {
                    state = OFFLINE_RIGHT;
                    // spdDiffI = 0;
                } else if (isFrontLeftBlack && isFrontRightBlack) {
                    state = STRAIGHT;
                }

                if (!isBackLeftBlack && !isBackRightBlack) {
                    atNode = false;
                }
            break;

            case OFFLINE_LEFT:
                if (isFrontCenterBlack) {
                    state = STRAIGHT_LEFT;
                }

                if (!isBackLeftBlack && !isBackRightBlack) {
                    atNode = false;
                }
            break;

            case OFFLINE_RIGHT:
                if (isFrontCenterBlack) {
                    state = STRAIGHT_RIGHT;
                }

                if (!isBackLeftBlack && !isBackRightBlack) {
                    atNode = false;
                }
            break;
        }

        double leftSpeed = 0, rightSpeed = 0;
        double leftFactor = 1, rightFactor = 1;

        // States actions
        switch (state) {
            case STRAIGHT:
                leftSpeed = MOTORS_INIT_SPEED;
                rightSpeed = MOTORS_INIT_SPEED;
            break;

            case STRAIGHT_LEFT:
                leftSpeed = MOTORS_INIT_SPEED;
                rightSpeed = MOTORS_INIT_SPEED / 2;
            break;

            case STRAIGHT_RIGHT:
                leftSpeed = MOTORS_INIT_SPEED / 2;
                rightSpeed = MOTORS_INIT_SPEED;
            break;

            case OFFLINE_LEFT:
                leftSpeed = MOTORS_INIT_SPEED / 2;
                rightSpeed = 0;
                rightFactor = 0;
                rightMotorController.reset();
            break;

            case OFFLINE_RIGHT:
                leftSpeed = 0;
                leftFactor = 0;
                leftMotorController.reset();
                rightSpeed = MOTORS_INIT_SPEED / 2;
            break;
        }

        double leftDistance, rightDistance;

        // Get motors throttle
        double leftPWM = leftMotorController.compute(elapsedTime, leftSpeed, leftDistance);
        double rightPWM = rightMotorController.compute(elapsedTime, rightSpeed, rightDistance);

        // Reduce distance
        distance -= max(leftDistance, rightDistance);

        // Stopping at nodes
        if ((isBackLeftBlack || wasBackLeftBlack) && !atNode && distance < STEP / 2) {
            if (state != 6) // Prevent stuck when state == 6 && right tire on black line
                leftPWM = 0;
            
            wasBackLeftBlack = true;
        }

        if ((isBackRightBlack || wasBackRightBlack) && !atNode && distance < STEP / 2) {
            if (state != 7)
                rightPWM = 0;

            wasBackRightBlack = true;
        }

        logs += ("MOVE:: elapsedTime: " + String(elapsedTime) + 
                 " - STATE: " + String(state) + 
                 " - atNode: " + String(atNode) + 
                 // " - LeftRPM: " + String(leftRPM) + 
                 " - LeftSpeed: " + String(leftSpeed) + 
                 " - LeftPWM: " + String(leftPWM * leftFactor) + 
                 // " - LeftP: " + String(leftMotorController.p) +
                 // " - LeftI: " + String(leftMotorController.i) +
                 // " - LeftD: " + String(leftMotorController.d) + 
                 // " - RightRPM: " + String(rightRPM) +
                 " - RightSpeed: " + String(rightSpeed) + 
                 " - RightPWM: " + String(rightPWM * rightFactor) +
                 // " - RightP: " + String(rightMotorController.p) +
                 // " - RightI: " + String(rightMotorController.i) +
                 // " - RightD: " + String(rightMotorController.d) + 
                 // " - leftDistance: " + String(leftDistance) + 
                 // " - rightDistance: " + String(rightDistance) + 
                 " - Distance: " + String(distance) + 
                 " - isFrontLeftBlack: " + String(isFrontLeftBlack) +
                 " - isFrontRightBlack: " + String(isFrontRightBlack) +
                 " - isFrontCenterBlack: " + String(isFrontCenterBlack) +
                 " - isBackLeftBlack: " + String(isBackLeftBlack) +
                 " - isBackRightBlack: " + String(isBackRightBlack) +
                 " - wasBackLeftBlack: " + String(wasBackLeftBlack) +
                 " - wasBackRightBlack: " + String(wasBackRightBlack));

        if (wasBackLeftBlack && wasBackRightBlack && !atNode && distance < STEP / 2) {
            adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);        

            state = IDLE;
            atNode = true;

            logs += (" - Stopping");

            return true;
        }

        adjustMotors(leftPWM * leftFactor, rightPWM * rightFactor, HIGH, LOW, HIGH, LOW);

        return false;
    }

    bool rotate(bool isFrontCenterBlack, bool isFrontLeftBlack, bool isFrontRightBlack, bool isBackLeftBlack, bool isBackRightBlack, String& logs) {
        //
        // Timer
        //
        double current = millis();
        
        if (current - time < MOTORS_ADJUST_DELTA && 
            prevIsFrontRightBlack == isFrontRightBlack && 
            prevIsFrontLeftBlack == isFrontLeftBlack && 
            prevIsFrontCenterBlack == isFrontCenterBlack && 
            prevIsBackRightBlack == isBackRightBlack && 
            prevIsBackLeftBlack == isBackLeftBlack)
            return false;

        double elapsedTime = (current - time) / 1000.0;
        time = current;

        //
        // Line following
        //
        prevIsFrontRightBlack = isFrontRightBlack;
        prevIsFrontLeftBlack = isFrontLeftBlack;
        prevIsFrontCenterBlack = isFrontCenterBlack;
        prevIsBackRightBlack = isBackRightBlack;
        prevIsBackLeftBlack = isBackLeftBlack;

        // FSM transitions
        switch (state) {
            case PRE_ROTATE_RIGHT:
                if (!isFrontRightBlack) {
                    state = PRE_ROTATE_RIGHT_2;
                }
            break;

            case PRE_ROTATE_LEFT:
                if (!isFrontLeftBlack) {
                    state = PRE_ROTATE_LEFT_2;
                }
            break;

            case PRE_ROTATE_RIGHT_2:
                if (!isFrontCenterBlack) {
                    state = ROTATE_RIGHT;
                }
            break;

            case PRE_ROTATE_LEFT_2:
                if (!isFrontCenterBlack) {
                    state = ROTATE_LEFT;
                }
            break;
            
            case ROTATE_RIGHT:
                if (isFrontRightBlack && !isFrontCenterBlack) {
                    state = POST_ROTATE_RIGHT;
                } else if (isFrontRightBlack && isFrontCenterBlack) {
                    state = IDLE;
                }
            break;
            
            case ROTATE_LEFT: 
                if (isFrontLeftBlack && !isFrontCenterBlack) {
                    state = POST_ROTATE_LEFT;
                } else if (isFrontLeftBlack && isFrontCenterBlack) {
                    state = IDLE;
                }
            break;

            case POST_ROTATE_RIGHT:
                if (isFrontCenterBlack || (wasBackLeftBlack && wasBackRightBlack)) {
                    state = IDLE;
                }
            break;

            case POST_ROTATE_LEFT:
                if (isFrontCenterBlack || (wasBackLeftBlack && wasBackRightBlack)) {
                    state = IDLE;
                }
            break;
        }

        double leftSpeed = 0, rightSpeed = 0;
        double leftFactor = 1, rightFactor = 1;

        // States actions
        switch (state) {
            case IDLE:
                adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);        
            
                leftMotorController.reset();
                rightMotorController.reset();

                logs += ("TURN:: - state: " + String(state) + 
                         " - isFrontLeftBlack: " + String(isFrontLeftBlack) +
                         " - isFrontRightBlack: " + String(isFrontRightBlack) +
                         " - isFrontCenterBlack: " + String(isFrontCenterBlack) +
                         " - isBackLeftBlack: " + String(isBackLeftBlack) +
                         " - isBackRightBlack: " + String(isBackRightBlack) +
                         " - wasBackLeftBlack: " + String(wasBackLeftBlack) +
                         " - wasBackRightBlack: " + String(wasBackRightBlack) +
                         " - elapsedTime: " + String(elapsedTime));

                return true;
            break;

            case PRE_ROTATE_RIGHT:
                rightSpeed = -MOTORS_ROTATION_SPEED;
                leftSpeed = MOTORS_ROTATION_SPEED;
            break;

            case PRE_ROTATE_LEFT:
                rightSpeed = MOTORS_ROTATION_SPEED;
                leftSpeed = -MOTORS_ROTATION_SPEED;
            break;

            case PRE_ROTATE_RIGHT_2:
                rightSpeed = -MOTORS_ROTATION_SPEED;
                leftSpeed = MOTORS_ROTATION_SPEED;
            break;

            case PRE_ROTATE_LEFT_2:
                rightSpeed = MOTORS_ROTATION_SPEED;
                leftSpeed = -MOTORS_ROTATION_SPEED;
            break;
            
            case ROTATE_RIGHT:
                rightSpeed = -MOTORS_ROTATION_SPEED;
                leftSpeed = MOTORS_ROTATION_SPEED;
            break;
            
            case ROTATE_LEFT:
                rightSpeed = MOTORS_ROTATION_SPEED;
                leftSpeed = -MOTORS_ROTATION_SPEED;
            break;
            
            case POST_ROTATE_RIGHT:
                rightSpeed = -MOTORS_ROTATION_SPEED;
                leftSpeed = MOTORS_ROTATION_SPEED;

                if (isBackLeftBlack) {
                    wasBackLeftBlack = true;
                }
                
                if (isBackRightBlack) {
                    wasBackRightBlack = true;
                }

                if (wasBackRightBlack) {
                    rightSpeed = 0;
                    rightFactor = 0;
                }

                if (wasBackLeftBlack) {
                    leftSpeed = 0;
                    leftFactor = 0;
                }
            break;
            
            case POST_ROTATE_LEFT:
                rightSpeed = MOTORS_ROTATION_SPEED;
                leftSpeed = -MOTORS_ROTATION_SPEED;

                if (isBackLeftBlack) {
                    wasBackLeftBlack = true;
                }
                
                if (isBackRightBlack) {
                    wasBackRightBlack = true;
                }

                if (wasBackRightBlack) {
                    rightSpeed = 0;
                    rightFactor = 0;
                }

                if (wasBackLeftBlack) {
                    leftSpeed = 0;
                    leftFactor = 0;
                }
            break;
        }

        // Get motors throttle
        double leftPWM = leftMotorController.compute(elapsedTime, abs(leftSpeed));
        double rightPWM = rightMotorController.compute(elapsedTime, abs(rightSpeed));

        logs += ("TURN:: - state: " + String(state) + 
                 " - isFrontLeftBlack: " + String(isFrontLeftBlack) +
                 " - isFrontRightBlack: " + String(isFrontRightBlack) +
                 " - isFrontCenterBlack: " + String(isFrontCenterBlack) +
                 " - isBackLeftBlack: " + String(isBackLeftBlack) +
                 " - isBackRightBlack: " + String(isBackRightBlack) +
                 " - wasBackLeftBlack: " + String(wasBackLeftBlack) +
                 " - wasBackRightBlack: " + String(wasBackRightBlack) +
                 " - elapsedTime: " + String(elapsedTime) + 
                 // " - LeftRPM: " + String(leftRPM) + 
                 " - LeftSpeed: " + String(abs(leftSpeed)) + 
                 " - LeftPWM: " + String(leftPWM) + 
                 // " - LeftP: " + String(leftMotorController.p) +
                 // " - LeftI: " + String(leftMotorController.i) +
                 // " - LeftD: " + String(leftMotorController.d) + 
                 // " - RightRPM: " + String(rightRPM) +
                 " - RightSpeed: " + String(abs(rightSpeed)) + 
                 " - RightPWM: " + String(rightPWM)
                 // " - RightP: " + String(rightMotorController.p) +
                 // " - RightI: " + String(rightMotorController.i) +
                 // " - RightD: " + String(rightMotorController.d)
                 );

        // Set directions
        int lDir1 = HIGH;
        int lDir2 = LOW;
        int rDir1 = HIGH;
        int rDir2 = LOW;
        
        if (leftSpeed < 0) {
            Utils::swap(&lDir1, &lDir2);
        }
        
        if (rightSpeed < 0) {
            Utils::swap(&rDir1, &rDir2);
        }

        adjustMotors(leftPWM * leftFactor, rightPWM * rightFactor, lDir1, lDir2, rDir1, rDir2);

        return false;
    }

    bool retreat(bool isFrontCenterBlack, bool isFrontLeftBlack, bool isFrontRightBlack, bool isBackLeftBlack, bool isBackRightBlack, String& logs) {
        //
        // Timer
        //
        double current = millis();
        
        if (current - time < MOTORS_ADJUST_DELTA && 
            prevIsFrontRightBlack == isFrontRightBlack && 
            prevIsFrontLeftBlack == isFrontLeftBlack && 
            prevIsFrontCenterBlack == isFrontCenterBlack && 
            prevIsBackRightBlack == isBackRightBlack && 
            prevIsBackLeftBlack == isBackLeftBlack)
            return false;

        double elapsedTime = (current - time) / 1000.0;
        time = current;

        //
        // Line following
        //
        prevIsFrontRightBlack = isFrontRightBlack;
        prevIsFrontLeftBlack = isFrontLeftBlack;
        prevIsFrontCenterBlack = isFrontCenterBlack;
        prevIsBackRightBlack = isBackRightBlack;
        prevIsBackLeftBlack = isBackLeftBlack;

        // FSM transitions
        switch (state) {
            case PRE_RETREAT:
                if (!isFrontCenterBlack && isFrontLeftBlack) {
                    state = RETREAT;
                }
            break;

            case RETREAT:
                if (isFrontCenterBlack) {
                    state = MOVE;
                }
            break;
        }

        double leftSpeed = 0, rightSpeed = 0;

        switch (state) {
            case MOVE:
                adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);        
            
                leftMotorController.reset();
                rightMotorController.reset();

                logs += ("RETREAT:: - state: " + String(state) + 
                         " - isFrontLeftBlack: " + String(isFrontLeftBlack) +
                         " - isFrontRightBlack: " + String(isFrontRightBlack) +
                         " - isFrontCenterBlack: " + String(isFrontCenterBlack) +
                         " - isBackLeftBlack: " + String(isBackLeftBlack) +
                         " - isBackRightBlack: " + String(isBackRightBlack) +
                         " - wasBackLeftBlack: " + String(wasBackLeftBlack) +
                         " - wasBackRightBlack: " + String(wasBackRightBlack) +
                         " - elapsedTime: " + String(elapsedTime));

                return true;
            break;

            case PRE_RETREAT:
                rightSpeed = -MOTORS_ROTATION_SPEED;
                leftSpeed = MOTORS_ROTATION_SPEED;
            break;

            case RETREAT:
                rightSpeed = -MOTORS_ROTATION_SPEED;
                leftSpeed = MOTORS_ROTATION_SPEED;
            break;
        }

        // Get motors throttle
        double leftPWM = leftMotorController.compute(elapsedTime, abs(leftSpeed));
        double rightPWM = rightMotorController.compute(elapsedTime, abs(rightSpeed));

        // Set directions
        int lDir1 = HIGH;
        int lDir2 = LOW;
        int rDir1 = HIGH;
        int rDir2 = LOW;
        
        if (leftSpeed < 0) {
            Utils::swap(&lDir1, &lDir2);
        }
        
        if (rightSpeed < 0) {
            Utils::swap(&rDir1, &rDir2);
        }

        adjustMotors(leftPWM, rightPWM, lDir1, lDir2, rDir1, rDir2);

        return false;
    }

    void adjustMotors(int leftPWM, int rightPWM, int lDir1, int lDir2, int rDir1, int rDir2) {
        // Lock the stopping wheels
        if (leftPWM < EPS) {
            lDir1 = lDir2 = HIGH;
            leftPWM = PWMRANGE;
        }

        if (rightPWM < EPS) {
            rDir1 = rDir2 = HIGH;
            rightPWM = PWMRANGE;
        }

        // Send motor signals
        analogWrite(LEFT_SPED, leftPWM);
        analogWrite(RGHT_SPED, rightPWM);

        motorsPCF->write(LEFT_DIR1, lDir1);
        motorsPCF->write(LEFT_DIR2, lDir2);
        motorsPCF->write(RGHT_DIR1, rDir1);
        motorsPCF->write(RGHT_DIR2, rDir2);
    }
};