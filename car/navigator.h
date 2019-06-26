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

        double pid = constrain(PWMRANGE / 6. + p + i + d, -PWMRANGE, PWMRANGE);

        double throttle = constrain(pid, 0, PWMRANGE);

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
    Navigator() : leftMotorController(1.2, 0.1, 0, MOTORS_INIT_SPEED),
                  rightMotorController(1.2, 0.1, 0, MOTORS_INIT_SPEED) {
    }

    void setup(PCF857x* pcf1, Encoder* len, Encoder* ren) {
        // Motor pins
        pinMode(LEFT_SPED, OUTPUT);
        pinMode(RGHT_SPED, OUTPUT);

        leftMotorController.en = len;
        rightMotorController.en = ren;

        motorsPCF = pcf1;
    }

    STATE getState() {
        return state;
    }

    ACTION getAction() {
        return action;
    }

    void navigate(unsigned long distance, double angle, bool isFrontCenterBlack, bool isFrontLeftBlack, bool isFrontRightBlack, bool isBackLeftBlack, bool isBackRightBlack, String& logs) {
        if (action == ACTION::MOVE) {
            move(distance, isFrontCenterBlack, isFrontLeftBlack, isFrontRightBlack, isBackLeftBlack, isBackRightBlack, logs);
        } else if (action == ACTION::ROTATE_RIGHT || action == ACTION::ROTATE_LEFT) {
            rotate(angle, isFrontCenterBlack, isFrontLeftBlack, isFrontRightBlack, isBackLeftBlack, isBackRightBlack, logs);
        } else if (action == ACTION::RETREAT) {
            retreat(angle, isFrontCenterBlack, isFrontLeftBlack, isFrontRightBlack, isBackLeftBlack, isBackRightBlack, logs);
        }
    }

    void prepare() {
        time = millis();

        leftMotorController.reset();
        rightMotorController.reset();

        wasBackRightBlack = false;
        wasBackLeftBlack = false;
    }

    void stop() {
        preIdleState = state;
        preIdleAction = action;
        preIdleRemainingDistance = remainingDistance;
        preIdleRemainingAngle = remainingAngle;

        leftMotorController.reset();
        rightMotorController.reset();

        state = IDLE;

        action = ACTION::NONE;

        adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);
    }

    void move() {
        prepare();

        remainingDistance = STEP;
        movedDistance = 0;

        if (preIdleAction == ACTION::MOVE) {
            state = preIdleState;

            if (!preIdleActionCompleted) {
                remainingDistance = preIdleRemainingDistance;
                movedDistance = STEP - preIdleRemainingDistance;
            }
        } else {
            state = MOVE;
        }

        action = ACTION::MOVE;

        preIdleActionCompleted = false;
    }

    void retreat(double angle) {
        prepare();

        remainingDistance = movedDistance;

        remainingAngle = 180;

        prevAngle = angle;

        state = PRE_RETREAT;

        action = ACTION::RETREAT;
    }

    void rotateLeft(double angle) {
        prepare();

        remainingAngle = 90;

        prevAngle = angle;

        state = PRE_ROTATE_LEFT;

        action = ACTION::ROTATE_LEFT;
    }

    void rotateRight(double angle) {
        prepare();

        remainingAngle = 90;

        prevAngle = angle;

        state = PRE_ROTATE_RIGHT;

        action = ACTION::ROTATE_RIGHT;
    }

private:
    STATE state = INIT;
    ACTION action = ACTION::NONE;

    PCF857x* motorsPCF;

    MotorController leftMotorController;
    MotorController rightMotorController;

    double remainingDistance = 0; // Remaining distance
    double time = 0;
    double movedDistance = 0;
    double remainingAngle = 0;
    double prevAngle = 0;

    STATE preIdleState = INIT; // The state before stopping the robot due to deactivation, ...etc
    ACTION preIdleAction = ACTION::NONE;
    double preIdleRemainingDistance = 0;
    double preIdleRemainingAngle = 0;
    bool preIdleActionCompleted = true;

    // Navigation flags
    bool atNode = true;
    bool prevIsFrontRightBlack = false;
    bool prevIsFrontLeftBlack = false;
    bool prevIsBackRightBlack = false;
    bool prevIsBackLeftBlack = false;
    bool prevIsFrontCenterBlack = false;
    bool wasBackLeftBlack = false;
    bool wasBackRightBlack = false;

    void move(unsigned long obstacleDistance, bool isFrontCenterBlack, bool isFrontLeftBlack, bool isFrontRightBlack, bool isBackLeftBlack, bool isBackRightBlack, String& logs) {
        // Emergency braking
        if (obstacleDistance < MIN_DISTANCE) {
            logs += ("Error: nearby object! " + String(obstacleDistance) + "\n");

            stop();

            return;
        }

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
            return;

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
                // if (action == ACTION::RETREAT && remainingAngle < 40) {
                //     state = STRAIGHT;
                // } else
                if (isFrontLeftBlack && isFrontRightBlack && isFrontCenterBlack) {
                    state = STRAIGHT;
                } else if (!isFrontLeftBlack && !isFrontRightBlack && isFrontCenterBlack) {
                    state = STRAIGHT;
                } else if (!isFrontLeftBlack && isFrontRightBlack && isFrontCenterBlack) {
                    state = STRAIGHT_LEFT;
                } else if (isFrontLeftBlack && !isFrontRightBlack && isFrontCenterBlack) {
                    state = STRAIGHT_RIGHT;
                } else if (!isFrontLeftBlack && isFrontRightBlack && !isFrontCenterBlack) {
                    state = OFFLINE_LEFT;
                } else if (isFrontLeftBlack && !isFrontRightBlack && !isFrontCenterBlack) {
                    state = OFFLINE_RIGHT;
                }
            break;

            case STRAIGHT:
                // if (action == ACTION::RETREAT && remainingAngle < 20) {
                //     state = STRAIGHT;
                // } else
                if (isFrontLeftBlack && isFrontRightBlack) {
                    state = STRAIGHT;
                } else if (isFrontLeftBlack && !isFrontRightBlack && isFrontCenterBlack) {
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
                } else if (isFrontLeftBlack && isFrontRightBlack) {
                    state = STRAIGHT;
                }

                if (!isBackLeftBlack && !isBackRightBlack) {
                    atNode = false;
                }
            break;

            case OFFLINE_LEFT:
                if (isFrontCenterBlack && isFrontRightBlack) {
                    state = STRAIGHT_LEFT;
                } else if (isFrontCenterBlack && isFrontLeftBlack) {
                    state = STRAIGHT_RIGHT;
                } else if (isFrontCenterBlack) {
                    state = STRAIGHT;
                }

                if (!isBackLeftBlack && !isBackRightBlack) {
                    atNode = false;
                }
            break;

            case OFFLINE_RIGHT:
                if (isFrontCenterBlack && isFrontLeftBlack) {
                    state = STRAIGHT_RIGHT;
                } else if (isFrontCenterBlack && isFrontRightBlack) {
                    state = STRAIGHT_LEFT;
                } else if (isFrontCenterBlack) {
                    state = STRAIGHT;
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

        // Update distances
        remainingDistance -= max(leftDistance, rightDistance);

        if (action != ACTION::RETREAT)
            movedDistance += max(leftDistance, rightDistance);

        int thresholdDistance = STEP / 2;

        if (action == ACTION::RETREAT) {
            thresholdDistance = movedDistance * 0.75;
        }

        // Stopping at nodes
        if ((isBackLeftBlack || wasBackLeftBlack) && !atNode && remainingDistance < thresholdDistance) {
            if (state != 6) // Prevent stuck when state == 6 && right tire on black line
                leftPWM = 0;

            wasBackLeftBlack = true;
        }

        if ((isBackRightBlack || wasBackRightBlack) && !atNode && remainingDistance < thresholdDistance) {
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
                 " - Remaining Distance: " + String(remainingDistance) +
                 " - Moved Distance: " + String(movedDistance / 2) +
                 " - isFrontLeftBlack: " + String(isFrontLeftBlack) +
                 " - isFrontRightBlack: " + String(isFrontRightBlack) +
                 " - isFrontCenterBlack: " + String(isFrontCenterBlack) +
                 " - isBackLeftBlack: " + String(isBackLeftBlack) +
                 " - isBackRightBlack: " + String(isBackRightBlack) +
                 " - wasBackLeftBlack: " + String(wasBackLeftBlack) +
                 " - wasBackRightBlack: " + String(wasBackRightBlack) + "\n\n");

        if (wasBackLeftBlack && wasBackRightBlack && remainingDistance < thresholdDistance && (!atNode || action == ACTION::RETREAT)) {
            // We've arrived sucessfully without interruptions.
            preIdleActionCompleted = true;

            logs += (" - Stopping");

            stop();

            return;
        }

        adjustMotors(leftPWM * leftFactor, rightPWM * rightFactor, HIGH, LOW, HIGH, LOW);

        return;
    }

    void rotate(double angle, bool isFrontCenterBlack, bool isFrontLeftBlack, bool isFrontRightBlack, bool isBackLeftBlack, bool isBackRightBlack, String& logs) {
        remainingAngle -= min(abs(angle - prevAngle), 360 - abs(angle - prevAngle));
        prevAngle = angle;

        // if (remainingAngle <= 0) { // Incosistent.
        //     state = IDLE;
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
            return;

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
        double leftFactor = 1, rightFactor = 1; // Rotation should be slower than normal movement.

        // States actions
        switch (state) {
            case IDLE:
                stop();

                logs += ("TURN:: - state: " + String(state) +
                         " - Angle: " + String(angle) +
                         " - isFrontLeftBlack: " + String(isFrontLeftBlack) +
                         " - isFrontRightBlack: " + String(isFrontRightBlack) +
                         " - isFrontCenterBlack: " + String(isFrontCenterBlack) +
                         " - isBackLeftBlack: " + String(isBackLeftBlack) +
                         " - isBackRightBlack: " + String(isBackRightBlack) +
                         " - wasBackLeftBlack: " + String(wasBackLeftBlack) +
                         " - wasBackRightBlack: " + String(wasBackRightBlack) +
                         " - elapsedTime: " + String(elapsedTime) + "\n\n");

                if (action == ACTION::RETREAT) {
                    state = MOVE;

                    return;
                }

                return;
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
                 " - Angle: " + String(angle) +
                 " - remainingAngle: " + String(remainingAngle) +
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
                 + "\n\n");

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

        return;
    }

    void retreat(double angle, bool isFrontCenterBlack, bool isFrontLeftBlack, bool isFrontRightBlack, bool isBackLeftBlack, bool isBackRightBlack, String& logs) {
        remainingAngle -= min(abs(angle - prevAngle), 360 - abs(angle - prevAngle));
        prevAngle = angle;

        if (remainingAngle <= 0) {
            logs += "RAngle: " + String(remainingAngle) + "\n\n";

            state = MOVE;
        }

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
            return;

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
            case PRE_RETREAT: // ToDo: handle all cases
                if (!isFrontCenterBlack && isFrontLeftBlack) {
                    state = RETREAT;
                }
            break;

            case RETREAT:
                if (isFrontCenterBlack) {
                    logs += "Front center: " + String(remainingAngle) + "\n\n";

                    if (remainingAngle < 40) {
                        state = MOVE;
                    } else {
                        state = PRE_ROTATE_RIGHT;
                    }
                }
            break;
        }

        double leftSpeed = 0, rightSpeed = 0;

        logs += ("RETREAT:: - state: " + String(state) +
                         " - Angle: " + String(angle) +
                         " - remainingAngle: " + String(remainingAngle) +
                         " - isFrontLeftBlack: " + String(isFrontLeftBlack) +
                         " - isFrontRightBlack: " + String(isFrontRightBlack) +
                         " - isFrontCenterBlack: " + String(isFrontCenterBlack) +
                         " - isBackLeftBlack: " + String(isBackLeftBlack) +
                         " - isBackRightBlack: " + String(isBackRightBlack) +
                         " - wasBackLeftBlack: " + String(wasBackLeftBlack) +
                         " - wasBackRightBlack: " + String(wasBackRightBlack) +
                         " - elapsedTime: " + String(elapsedTime) + "\n\n");

        switch (state) {
            case PRE_ROTATE_RIGHT:
            case MOVE:
                adjustMotors(PWMRANGE, PWMRANGE, HIGH, HIGH, HIGH, HIGH);

                leftMotorController.reset();
                rightMotorController.reset();

                return;
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

        return;
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