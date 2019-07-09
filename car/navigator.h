#pragma once

#include "communicator.h"
#include "motor_controller.h"
#include "sensors/encoder.h"
#include "utils/utils.h"
#include "utils/constants.h"

class Navigator {

public:

    Navigator() : leftMotorController(LEFT_KP, LEFT_KI, LEFT_KD, LEFT_INIT_THROTTLE,
                                      LEFT_SPED, LEFT_DIR1, LEFT_DIR2),
                  rightMotorController(RIGHT_KP, RIGHT_KI, RIGHT_KD, RIGHT_INIT_THROTTLE,
                                       RIGHT_SPED, RIGHT_DIR1, RIGHT_DIR2) {
    }

    void setup(PCF857x* pcf1, Encoder* len, Encoder* ren, bool* blockedPtr, String* logsPtr) {
        // Configure motors controllers
        leftMotorController.en = len;
        rightMotorController.en = ren;

        leftMotorController.pcf = pcf1;
        rightMotorController.pcf = pcf1;

        blocked = blockedPtr;

        // Logs
        logs = logsPtr;
    }

    void navigate(double obstacleDistance, double angle, bool blackSensors[], ExecutionState& execState) {
        if (executionState.state != EXECUTION_STATE::ONGOING) {
            return;
        }

        if (action == ACTION::MOVE) {
            move(obstacleDistance, angle, blackSensors);
        } else if (action == ACTION::ROTATE_RIGHT || action == ACTION::ROTATE_LEFT) {
            rotate(angle, blackSensors);
        } else if (action == ACTION::RETREAT) {
            retreat(obstacleDistance, angle, blackSensors);
        }

        execState.state = executionState.state;
        execState.error = executionState.error;

        // We've finished the last order and now we are waiting for the next one.
        if (executionState.state == EXECUTION_STATE::FINISHED) {
            executionState.state = EXECUTION_STATE::IDLE;

            logs->concat("Finished\n");
        }
    }

    void prepare() {
        leftMotorController.reset();
        rightMotorController.reset();

        blackLeftWasNotBlack = false;
        blackRightWasNotBlack = false;
        stopLeftMotor = false;
        stopRightMotor = false;

        postRotateHold = false;
        reachedNode = false;
    }

    void config() {
        prepare();

        action = ACTION::NONE;
        moveState = MOVE_STATE::NONE;
        rotateState = ROTATE_STATE::NONE;
        retreatState = RETREAT_STATE::NONE;

        executionState.state = EXECUTION_STATE::IDLE;

        init();

        logs->concat("Configed\n");
    }

    void stop() {
        if (executionState.state == EXECUTION_STATE::IDLE) {
            return;
        }

        pause();

        logs->concat("Stopped\n");
    }

    void terminate() {
        halt(EXECUTION_ERROR::EXCEEDED_ACTION_PERIOD);

        logs->concat("Terminated\n");
    }

    void move(double angle, bool recover = false) {
        if (executionState.state == EXECUTION_STATE::IDLE && !recover) {
            prepare();

            // Recover from an incomplete rotation
            if (action == ACTION::ROTATE_LEFT) {
                if (rotateState == ROTATE_STATE::FAILED_ROTATE_LEFT_OFFLINE_LEFT) {
                    moveState = MOVE_STATE::DRIFTING_LEFT;
                } else if (rotateState == ROTATE_STATE::FAILED_ROTATE_LEFT_OFFLINE_RIGHT) {
                    moveState = MOVE_STATE::DRIFTING_RIGHT;
                } else {
                    moveState = MOVE_STATE::STRAIGHT;
                }
            } else if (action == ACTION::ROTATE_RIGHT) {
                if (rotateState == ROTATE_STATE::FAILED_ROTATE_RIGHT_OFFLINE_LEFT) {
                    moveState = MOVE_STATE::DRIFTING_LEFT;
                } else if (rotateState == ROTATE_STATE::FAILED_ROTATE_RIGHT_OFFLINE_RIGHT) {
                    moveState = MOVE_STATE::DRIFTING_RIGHT;
                } else {
                    moveState = MOVE_STATE::STRAIGHT;
                }
            }

            action = ACTION::MOVE;
            executionState.state = EXECUTION_STATE::ONGOING;

            leftMotorController.setSpeedIncrementStep(MOTORS_MOVE_SPEED_INCREMENT);
            rightMotorController.setSpeedIncrementStep(MOTORS_MOVE_SPEED_INCREMENT);

            logs->concat("Start moving\n");
            logs->concat("angle: " + String(angle) + " - referenceAngle: " + String(referenceAngle) + " \n");
        } else if (executionState.state == EXECUTION_STATE::PAUSE && action == ACTION::MOVE && recover) {
            executionState.state = EXECUTION_STATE::ONGOING;

            logs->concat("Resume moving\n");
        }
    }

    void rotateRight(double angle, bool recover = false) {
        if (executionState.state == EXECUTION_STATE::IDLE && !recover) {
            prepare();

            rotateState = ROTATE_STATE::PREPARE_ROTATE_RIGHT;
            previousAngle = angle;
            referenceAngle = int(referenceAngle + 90) % 360;
            remainingAngle = Utils::anglesSmallDifference(angle, referenceAngle);

            if (action == ACTION::MOVE) {
                if (moveState == MOVE_STATE::OFFLINE_RIGHT) {
                    rotateState = ROTATE_STATE::ROTATE_RIGHT;
                }
            } else if (action == ACTION::RETREAT) {
                if (retreatState == RETREAT_STATE::POST_RETREAT_MOVE &&
                    moveState == MOVE_STATE::OFFLINE_RIGHT) {
                    rotateState = ROTATE_STATE::ROTATE_RIGHT;
                }
            }

            action = ACTION::ROTATE_RIGHT;
            executionState.state = EXECUTION_STATE::ONGOING;

            leftMotorController.setSpeedIncrementStep(MOTORS_ROTATE_SPEED_INCREMENT);
            rightMotorController.setSpeedIncrementStep(MOTORS_ROTATE_SPEED_INCREMENT);

            logs->concat("Start rotating right\n");
            logs->concat("angle: " + String(angle) + "\n");
        } else if (executionState.state == EXECUTION_STATE::PAUSE && action == ACTION::ROTATE_RIGHT && recover) {
            executionState.state = EXECUTION_STATE::ONGOING;

            logs->concat("Resume rotating right\n");
        }
    }

    void rotateLeft(double angle, bool recover = false) {
        if (executionState.state == EXECUTION_STATE::IDLE && !recover) {
            prepare();

            rotateState = ROTATE_STATE::PREPARE_ROTATE_LEFT;
            previousAngle = angle;
            referenceAngle = int(referenceAngle - 90 + 360) % 360;
            remainingAngle = Utils::anglesSmallDifference(angle, referenceAngle);

            if (action == ACTION::MOVE) {
                if (moveState == MOVE_STATE::OFFLINE_LEFT) {
                    rotateState = ROTATE_STATE::ROTATE_LEFT;
                }
            } else if (action == ACTION::RETREAT) {
                if (retreatState == RETREAT_STATE::POST_RETREAT_MOVE &&
                    moveState == MOVE_STATE::OFFLINE_LEFT) {
                    rotateState = ROTATE_STATE::ROTATE_LEFT;
                }
            }

            action = ACTION::ROTATE_LEFT;
            executionState.state = EXECUTION_STATE::ONGOING;

            leftMotorController.setSpeedIncrementStep(MOTORS_ROTATE_SPEED_INCREMENT);
            rightMotorController.setSpeedIncrementStep(MOTORS_ROTATE_SPEED_INCREMENT);

            logs->concat("Start rotating left\n");
            logs->concat("angle: " + String(angle) + "\n");
        } else if (executionState.state == EXECUTION_STATE::PAUSE && action == ACTION::ROTATE_LEFT && recover) {
            executionState.state = EXECUTION_STATE::ONGOING;

            logs->concat("Resume rotating left\n");
        }
    }

    void retreat(double angle, bool recover = false) {
        logs->concat(String((int) executionState.state) + " - " + String((int) action) + " " + String(recover));

        if (executionState.state == EXECUTION_STATE::PAUSE && action == ACTION::MOVE && !recover) {
            // Store the moved distance before resetting the controllers
            double leftDistance = leftMotorController.getTotalDistance();
            double rightDistance = rightMotorController.getTotalDistance();

            double movedDistance = (leftDistance + rightDistance) / 2.0;

            // Reset
            prepare();

            action = ACTION::RETREAT;
            retreatState = RETREAT_STATE::RETREAT;
            executionState.state = EXECUTION_STATE::ONGOING;

            // Angle that the robot should rotate to first
            // We will always rotate right
            previousAngle = angle;
            referenceAngle = int(referenceAngle + 180) % 360;

            if (moveState == MOVE_STATE::STRAIGHT || moveState == MOVE_STATE::ALIGNMENT) {
                remainingAngle = Utils::anglesAverageDifference(angle, referenceAngle);
            } else if (moveState == MOVE_STATE::STRAIGHT_RIGHT ||
                       moveState == MOVE_STATE::DRIFTING_RIGHT ||
                       moveState == MOVE_STATE::OFFLINE_RIGHT) {
                remainingAngle = Utils::anglesSmallDifference(angle, referenceAngle);
            } else if (moveState == MOVE_STATE::STRAIGHT_LEFT ||
                       moveState == MOVE_STATE::DRIFTING_LEFT ||
                       moveState == MOVE_STATE::OFFLINE_LEFT) {
                remainingAngle = Utils::anglesLargeDifference(angle, referenceAngle);
            }

            postRetreatDistance = movedDistance;

            leftMotorController.setSpeedIncrementStep(MOTORS_MOVE_SPEED_INCREMENT / 2);
            rightMotorController.setSpeedIncrementStep(MOTORS_MOVE_SPEED_INCREMENT / 2);

            logs->concat("Start retreating\n");
            logs->concat("MS: " + String((int) moveState) + " Current angle: " + String(angle) + " refA: " + referenceAngle + " ra: " + String(remainingAngle) + "\n");
        } else if (executionState.state == EXECUTION_STATE::PAUSE && action == ACTION::RETREAT && recover) {
            executionState.state = EXECUTION_STATE::ONGOING;

            logs->concat("Resume reterating\n");
        }
    }

    void setReferenceAngle(double angle) {
        referenceAngle = angle;

        logs->concat("Setting referenceAngle: " + String(angle) + "\n");
    }

    double getReferenceAngle() {
        return referenceAngle;
    }

private:

    // ====================
    // Members

    String* logs;

    MotorController leftMotorController;
    MotorController rightMotorController;

    // State variables
    ACTION action = ACTION::NONE;

    ExecutionState executionState;
    MOVE_STATE moveState = MOVE_STATE::NONE;
    ROTATE_STATE rotateState = ROTATE_STATE::NONE;
    RETREAT_STATE retreatState = RETREAT_STATE::NONE;

    bool blackLeftWasNotBlack = false; // Registers when sensor leaves black line
    bool blackRightWasNotBlack = false; // Registers when sensor leaves black line
    bool stopLeftMotor = false;
    bool stopRightMotor = false;

    bool reachedNode = false;
    bool* blocked;

    double referenceAngle;

    double remainingAngle;
    double previousAngle;

    double postRetreatDistance;

    double time = millis();

    double holdStartTime = 0;

    bool postRotateHold = false;
    double postRotateHoldStartTime = 0;

    bool monitorBlockage = false;
    double blockageTime = 0;

    // ====================
    // Execution state updating function

    void init() {
        executionState.state = EXECUTION_STATE::IDLE;

        hold();

        logs->concat("Init\n");
    }

    void pause() {
        executionState.state = EXECUTION_STATE::PAUSE;

        hold();

        logs->concat("Pausing\n");
    }

    void done() {
        executionState.state = EXECUTION_STATE::FINISHED;

        hold();

        logs->concat("Done\n");
    }

    void halt(EXECUTION_ERROR error) {
        executionState.state = EXECUTION_STATE::ERROR;
        executionState.error = error;

        hold();

        logs->concat("Halting due: " + String((int) error) + "\n");
    }

    void hold() {
        leftMotorController.brake();
        rightMotorController.brake();

        logs->concat("Holding motors\n");
    }

    // ====================
    // Moving handling functions

    // ToDo
    bool prevIsFrontLeftBlack = false;
    bool prevIsFrontCenterBlack = false;
    bool prevIsFrontRightBlack = false;
    bool prevIsBackLeftBlack = false;
    bool prevIsBackRightBlack = false;

    void move(double obstacleDistance, double angle, bool blackSensors[]) {
        // Emergency braking
        if (obstacleDistance < MIN_DISTANCE) {
            if (!monitorBlockage) {
                logs->concat("Error: starting monitoring obstacle at: " + String(obstacleDistance) + "\n");

                monitorBlockage = true;
                blockageTime = millis();
            } else if (millis() - blockageTime > 20) {
                logs->concat("Error: obstacle at: " + String(obstacleDistance) + "\n");

                *blocked = true;

                pause();

                return;
            }
        } else {
            monitorBlockage = false;
            blockageTime = 0;
        }

        // Update remaining distances
        double leftDistance = leftMotorController.getTotalDistance();
        double rightDistance = rightMotorController.getTotalDistance();

        double remainingDistance = STEP - (leftDistance + rightDistance) / 2.0;
        double minimumDistanceToNode = STEP / 4.0;

        if (remainingDistance < -EXCESS_DISTANCE_LIMIT) {
            logs->concat("Error: excess distance: remainingDistance: " + String(remainingDistance) + " - " +
                         " leftDistance: " + leftDistance + " - rightDistance: " + rightDistance + "\n");

            halt(EXECUTION_ERROR::EXCEEDED_ALLOWED_DISTANCE);

            return;
        }

        // FSM transitions
        updateMoveState(angle, blackSensors, remainingDistance, minimumDistanceToNode);

        // Get motors new speeds
        double leftSpeed, rightSpeed;
        getMotorsMoveSpeeds(angle, leftSpeed, rightSpeed);

        // Update motors controllers
        leftMotorController.setSpeed(leftSpeed);
        rightMotorController.setSpeed(rightSpeed);

        leftMotorController.update();
        rightMotorController.update();

        // Compensate back black sensors errors
        // double diff = Utils::mapAngle(angle - referenceAngle);

        // if (stopLeftMotor && !stopRightMotor && diff < -10 && remainingDistance < STEP / 5) {
        //     stopRightMotor = true;
        //     logs->concat("Force stopping right motor, diff: " + String(diff) + "\n");
        // }

        // if (!stopLeftMotor && stopRightMotor && diff > 10 && remainingDistance < STEP / 5) {
        //     stopLeftMotor = true;
        //     logs->concat("Force stopping left motor, diff: " + String(diff) + "\n");
        // }

        // Check if arrived
        if (stopLeftMotor && stopRightMotor) {
            done();
        }

        // Logging
        if (millis() - time > 200 || prevIsFrontLeftBlack != blackSensors[0] ||
            prevIsFrontCenterBlack != blackSensors[1] ||
            prevIsFrontRightBlack != blackSensors[2] ||
            prevIsBackLeftBlack != blackSensors[3] ||
            prevIsBackRightBlack != blackSensors[4]) {

            time = millis();

            prevIsFrontLeftBlack = blackSensors[0];
            prevIsFrontCenterBlack = blackSensors[1];
            prevIsFrontRightBlack = blackSensors[2];
            prevIsBackLeftBlack = blackSensors[3];
            prevIsBackRightBlack = blackSensors[4];

            logs->concat("MS: " + String((int) moveState) + "\n");
            logs->concat("a: " + String(angle) + "\n");
            logs->concat("rn: " + String(reachedNode) + "\n");
            logs->concat("RefA: " + String(referenceAngle) + "\n");
            // logs->concat("lspd: " + String(leftMotorController.getSpeed()) + "\n");
            // logs->concat("rspd: " + String(rightMotorController.getSpeed()) + "\n");
            // logs->concat("lspeed: " + String(leftSpeed) + "\n");
            // logs->concat("rspeed: " + String(rightSpeed) + "\n");
            // logs->concat("rd: " + String(remainingDistance) + "\n");
            // logs->concat("lp: " + String(leftMotorController.p) + "\n");
            // logs->concat("li: " + String(leftMotorController.i) + "\n");
            // logs->concat("lpwm: " + String(leftMotorController.PWM) + "\n");
            // logs->concat("rp: " + String(rightMotorController.p) + "\n");
            // logs->concat("ri: " + String(rightMotorController.i) + "\n");
            // logs->concat("rpwm: " + String(rightMotorController.PWM) + "\n");
            // logs->concat("lrpm: " + String(leftMotorController.RPM) + "\n");
            // logs->concat("rrpm: " + String(rightMotorController.RPM) + "\n");
            // logs->concat("leftd: " + String(leftMotorController.getTotalDistance()) + "\n");
            // logs->concat("rghtd: " + String(rightMotorController.getTotalDistance()) + "\n");
            // logs->concat("mdtn: " + String(minimumDistanceToNode) + "\n");
            // logs->concat("blnb: " + String(blackLeftWasNotBlack) + "\n");
            // logs->concat("brnb: " + String(blackRightWasNotBlack) + "\n");
            logs->concat("slm: " + String(stopLeftMotor) + "\n");
            logs->concat("srm: " + String(stopRightMotor) + "\n");
            logs->concat("blks: " + String(blackSensors[0]) + " " +
                                    String(blackSensors[1]) + " " +
                                    String(blackSensors[2]) + " " +
                                    String(blackSensors[3]) + " " +
                                    String(blackSensors[4]) + " " +
                                    "\n\n\n");
        }
    }

    void updateMoveState(double angle, bool blackSensors[], double remainingDistance, double minimumDistanceToNode) {
        bool isFrontLeftBlack = blackSensors[0];
        bool isFrontCenterBlack = blackSensors[1];
        bool isFrontRightBlack = blackSensors[2];
        bool isBackLeftBlack = blackSensors[3];
        bool isBackRightBlack = blackSensors[4];

        if (!isBackLeftBlack) {
            blackLeftWasNotBlack = true;
        }

        if (!isBackRightBlack) {
            blackRightWasNotBlack = true;
        }

        // Stopping at nodes
        if (remainingDistance < minimumDistanceToNode) {
            stopLeftMotor |= (blackRightWasNotBlack && isBackLeftBlack);
            stopRightMotor |= (blackRightWasNotBlack && isBackRightBlack);
        }

        double diff = Utils::mapAngle(angle - referenceAngle);

        // if (reachedNode && moveState == MOVE_STATE::ALIGNMENT &&
        //     (isFrontCenterBlack || isFrontLeftBlack || isFrontRightBlack)) {
        //     if (diff < -5) {
        //         moveState = MOVE_STATE::DRIFTING_LEFT;
        //     } else if (diff > 5) {
        //         moveState = MOVE_STATE::DRIFTING_RIGHT;
        //     }

        //     return;
        // }

        // ToDo: when we come back again, the state will change and overrides
        // the above code.

        // FSM transitions
        if (isFrontCenterBlack && isFrontLeftBlack && isFrontRightBlack) {
            moveState = MOVE_STATE::STRAIGHT;

            if (remainingDistance <= STEP / 3) {
                moveState = MOVE_STATE::ALIGNMENT;

                reachedNode = true;
            } else if (remainingDistance > STEP * 0.75) {
                moveState = MOVE_STATE::ALIGNMENT;
            }
        } else if (isFrontCenterBlack && isFrontLeftBlack && !isFrontRightBlack) {
            moveState = MOVE_STATE::STRAIGHT_RIGHT;
        } else if (isFrontCenterBlack && !isFrontLeftBlack && isFrontRightBlack) {
            moveState = MOVE_STATE::STRAIGHT_LEFT;
        } else if (isFrontCenterBlack && !isFrontLeftBlack && !isFrontRightBlack) {
            moveState = MOVE_STATE::STRAIGHT;
        } else if (!isFrontCenterBlack && isFrontLeftBlack && isFrontRightBlack) {
            moveState = MOVE_STATE::ALIGNMENT;
        } else if (!isFrontCenterBlack && isFrontLeftBlack && !isFrontRightBlack) {
            moveState = MOVE_STATE::DRIFTING_RIGHT;
        } else if (!isFrontCenterBlack && !isFrontLeftBlack && isFrontRightBlack) {
            moveState = MOVE_STATE::DRIFTING_LEFT;
        } else if (!isFrontCenterBlack && !isFrontLeftBlack && !isFrontRightBlack) {
            if (moveState == MOVE_STATE::DRIFTING_RIGHT || moveState == MOVE_STATE::STRAIGHT_RIGHT) {
                moveState = MOVE_STATE::OFFLINE_RIGHT;
            } else if (moveState == MOVE_STATE::DRIFTING_LEFT || moveState == MOVE_STATE::STRAIGHT_LEFT) {
                moveState = MOVE_STATE::OFFLINE_LEFT;
            } else if (moveState == MOVE_STATE::STRAIGHT && abs(diff) < EPS) {
                logs->concat("ERROR: angle: " + String(angle) + " - referenceAngle: " + String(referenceAngle) + " - diff: " + String(diff) + "\n");

                halt(EXECUTION_ERROR::UNKNOWN);
            }
        }

        double leftDiff = Utils::anglesSmallDifference(angle, int(referenceAngle - 90 + 360) % 360);
        double rightDiff = Utils::anglesSmallDifference(angle, int(referenceAngle + 90) % 360);

        if (moveState == MOVE_STATE::OFFLINE_LEFT && diff > 30) {
            logs->concat("Error: 1 - ld: " + String(leftDiff) + " 0 rd: " + String(rightDiff) + " !\n");

            if (leftDiff > rightDiff) {
                logs->concat("S\n");

                halt(EXECUTION_ERROR::UNKNOWN);

                // moveState = MOVE_STATE::OFFLINE_RIGHT;
            }
        } else if (moveState == MOVE_STATE::OFFLINE_RIGHT && diff > 40) {
            logs->concat("Error: 2 - ld: " + String(leftDiff) + " 0 rd: " + String(rightDiff) + " !\n");

            if (leftDiff < rightDiff) {
                logs->concat("S\n");

                halt(EXECUTION_ERROR::UNKNOWN);

                // moveState = MOVE_STATE::OFFLINE_LEFT;
            }
        }
    }

    void getMotorsMoveSpeeds(double angle, double& leftSpeed, double& rightSpeed) {
        if (!reachedNode) {
            switch (moveState) {
                case MOVE_STATE::HOLD:
                    leftSpeed = 0;
                    rightSpeed = 0;
                break;

                case MOVE_STATE::STRAIGHT:
                    leftSpeed = MOTORS_SPEED;
                    rightSpeed = MOTORS_SPEED;
                break;

                case MOVE_STATE::STRAIGHT_LEFT:
                    leftSpeed = MOTORS_SPEED;
                    rightSpeed = MOTORS_SPEED * C2;
                break;

                case MOVE_STATE::STRAIGHT_RIGHT:
                    leftSpeed = MOTORS_SPEED * C2;
                    rightSpeed = MOTORS_SPEED;
                break;

                case MOVE_STATE::DRIFTING_LEFT:
                    leftSpeed = MOTORS_SPEED * C1;
                    rightSpeed = MOTORS_SPEED * C4;
                break;

                case MOVE_STATE::DRIFTING_RIGHT:
                    leftSpeed = MOTORS_SPEED * C4;
                    rightSpeed = MOTORS_SPEED * C1;
                break;

                case MOVE_STATE::OFFLINE_LEFT:
                    leftSpeed = MOTORS_SPEED * C3;
                    rightSpeed = 0;
                break;

                case MOVE_STATE::OFFLINE_RIGHT:
                    leftSpeed = 0;
                    rightSpeed = MOTORS_SPEED * C3;
                break;

                case MOVE_STATE::ALIGNMENT:
                    getMotorsAlignmentSpeeds(angle, leftSpeed, rightSpeed);
                break;
            }
        } else {
            getMotorsAlignmentSpeeds(angle, leftSpeed, rightSpeed);
        }

        // Stopping at nodes
        if (stopLeftMotor && moveState != MOVE_STATE::OFFLINE_LEFT &&
            !((moveState == MOVE_STATE::ALIGNMENT || reachedNode) && rightSpeed < EPS)) {
            leftSpeed = 0;
        }

        if (stopRightMotor && moveState != MOVE_STATE::OFFLINE_RIGHT &&
            !((moveState == MOVE_STATE::ALIGNMENT || reachedNode) && leftSpeed < EPS)) {
            rightSpeed = 0;
        }
    }

    void getMotorsAlignmentSpeeds(double angle, double& leftSpeed, double& rightSpeed) {
        double diff = Utils::mapAngle(angle - referenceAngle);

        if (diff < -10) { // (-oo, -10)
            leftSpeed = MOTORS_SPEED * 0.5;
            rightSpeed = MOTORS_SPEED * 0;
        } else if (diff < -3) { // [-10, -3)
            leftSpeed = MOTORS_SPEED * 0.8;
            rightSpeed = MOTORS_SPEED * 0.5;
        } else if (diff > 10) { // (3, 10]
            leftSpeed = MOTORS_SPEED * 0;
            rightSpeed = MOTORS_SPEED * 0.5;
        } else if (diff > 3) { // (10, oo)
            leftSpeed = MOTORS_SPEED * 0.5;
            rightSpeed = MOTORS_SPEED * 0.8;
        } else { // [-3, 3]
            leftSpeed = MOTORS_SPEED * 0.8;
            rightSpeed = MOTORS_SPEED * 0.8;
        }
    }

    // ====================
    // Rotation handling functions

    void rotate(double angle, bool blackSensors[]) {
        if (Utils::anglesSmallDifference(angle, previousAngle) > 30) {
            logs->concat("Excessive angles: " + String(angle) +
                         " - previousAngle:" + String(previousAngle) + "\n");

            halt(EXECUTION_ERROR::EXCESSIVE_ANGLES);
        }

        // Update the remaining angle
        remainingAngle -= Utils::anglesSmallDifference(angle, previousAngle);
        previousAngle = angle;

        if (postRotateHold) {
            if (millis() - postRotateHoldStartTime > 500) {
                logs->concat("Last angle: " + String(angle) + "\n");

                done();

                updateRotateFinalState(remainingAngle, blackSensors);
            }
        }

        // FSM transitions
        updateRotateState(remainingAngle, blackSensors);

        // Get motors new speeds
        double leftSpeed, rightSpeed;
        getMotorsRotateSpeeds(leftSpeed, rightSpeed);

        // Update motors controllers
        leftMotorController.setSpeed(leftSpeed);
        rightMotorController.setSpeed(rightSpeed);

        leftMotorController.update();
        rightMotorController.update();

        // Call done if the limit is reached.
        // Here the moveState will indicate the current situation,
        // this info is helpful for the next action if it's a move action.
        // It will know that the rotation went wrong.
        if (!postRotateHold && isRotationDone(remainingAngle, blackSensors)) {
            postRotateHoldStartTime = millis();
            postRotateHold = true;

            hold();

            logs->concat("Post rotation hold started\n");
        }

        // Logging
        if (millis() - time > 200 || prevIsFrontLeftBlack != blackSensors[0] ||
            prevIsFrontCenterBlack != blackSensors[1] ||
            prevIsFrontRightBlack != blackSensors[2] ||
            prevIsBackLeftBlack != blackSensors[3] ||
            prevIsBackRightBlack != blackSensors[4] || isRotationDone(remainingAngle, blackSensors)) {

            time = millis();

            prevIsFrontLeftBlack = blackSensors[0];
            prevIsFrontCenterBlack = blackSensors[1];
            prevIsFrontRightBlack = blackSensors[2];
            prevIsBackLeftBlack = blackSensors[3];
            prevIsBackRightBlack = blackSensors[4];

            logs->concat("RS: " + String((int) rotateState) + "\n");
            logs->concat("a: " + String(angle) + "\n");
            logs->concat("ra: " + String(remainingAngle) + "\n");
            // logs->concat("lspeed: " + String(leftSpeed) + "\n");
            // logs->concat("rspeed: " + String(rightSpeed) + "\n");
            // logs->concat("lspd: " + String(leftMotorController.getSpeed()) + "\n");
            // logs->concat("rspd: " + String(rightMotorController.getSpeed()) + "\n");
            // logs->concat("ld: " + String(leftMotorController.getTotalDistance()) + "\n");
            // logs->concat("rd: " + String(rightMotorController.getTotalDistance()) + "\n");
            // logs->concat("lrpm: " + String(leftMotorController.getRPM()) + "\n");
            // logs->concat("rrpm: " + String(rightMotorController.getRPM()) + "\n");;
            // logs->concat("lpwm: " + String(leftMotorController.getPWM()) + "\n");
            // logs->concat("rpwm: " + String(rightMotorController.getPWM()) + "\n");
            // logs->concat("blnb: " + String(blackLeftWasNotBlack) + "\n");
            // logs->concat("brnb: " + String(blackRightWasNotBlack) + "\n");
            logs->concat("slm: " + String(stopLeftMotor) + "\n");
            logs->concat("srm: " + String(stopRightMotor) + "\n");
            logs->concat("blks: " + String(blackSensors[0]) + " " +
                                    String(blackSensors[1]) + " " +
                                    String(blackSensors[2]) + " " +
                                    String(blackSensors[3]) + " " +
                                    String(blackSensors[4]) + " " +
                                    "\n\n\n");
        }
    }

    void updateRotateState(double remainingAngle, bool blackSensors[]) {
        bool isFrontLeftBlack = blackSensors[0];
        bool isFrontCenterBlack = blackSensors[1];
        bool isFrontRightBlack = blackSensors[2];
        bool isBackLeftBlack = blackSensors[3];
        bool isBackRightBlack = blackSensors[4];

        // Distances
        double leftDistance = leftMotorController.getTotalDistance();
        double rightDistance = rightMotorController.getTotalDistance();

        double leftRPM = leftMotorController.getRPM();
        double rightRPM = rightMotorController.getRPM();

        if (!isBackLeftBlack && leftDistance > 20 && leftRPM > 0) {
            blackLeftWasNotBlack = true;
        }

        if (!isBackRightBlack && rightDistance > 20 && rightRPM > 0) {
            blackRightWasNotBlack = true;
        }

        // Stopping after rotation or
        // when one wheel spins much more than the expected rotation distance
        stopLeftMotor |= blackLeftWasNotBlack && isBackLeftBlack;
        stopRightMotor |= blackRightWasNotBlack && isBackRightBlack;

        if (postRotateHold) {
            stopLeftMotor = true;
            stopRightMotor = true;
        }

        switch (rotateState) {
            case ROTATE_STATE::PREPARE_ROTATE_RIGHT:
                if (!isFrontRightBlack && remainingAngle <= 90) {
                    rotateState = ROTATE_STATE::PRE_ROTATE_RIGHT;
                }
            break;

            case ROTATE_STATE::PREPARE_ROTATE_LEFT:
                if (!isFrontLeftBlack) {
                    rotateState = ROTATE_STATE::PRE_ROTATE_LEFT;
                }
            break;

            case ROTATE_STATE::PRE_ROTATE_RIGHT:
                if (!isFrontCenterBlack) {
                    rotateState = ROTATE_STATE::ROTATE_RIGHT;
                }
            break;

            case ROTATE_STATE::PRE_ROTATE_LEFT:
                if (!isFrontCenterBlack) {
                    rotateState = ROTATE_STATE::ROTATE_LEFT;
                }
            break;

            case ROTATE_STATE::ROTATE_RIGHT:
                if (isFrontRightBlack && !isFrontCenterBlack) {
                    rotateState = ROTATE_STATE::POST_ROTATE_RIGHT;
                } else if (!isFrontRightBlack && isFrontCenterBlack) {
                    rotateState = ROTATE_STATE::FINISH_ROTATE_RIGHT;
                }
            break;

            case ROTATE_STATE::ROTATE_LEFT:
                if (isFrontLeftBlack && !isFrontCenterBlack) {
                    rotateState = ROTATE_STATE::POST_ROTATE_LEFT;
                } else if (!isFrontLeftBlack && isFrontCenterBlack) {
                    rotateState = ROTATE_STATE::FINISH_ROTATE_LEFT;
                }
            break;

            case ROTATE_STATE::POST_ROTATE_RIGHT:
                if (!isFrontRightBlack) {
                    rotateState = ROTATE_STATE::FINISH_ROTATE_RIGHT;
                }
            break;

            case ROTATE_STATE::POST_ROTATE_LEFT:
                if (!isFrontLeftBlack) {
                    rotateState = ROTATE_STATE::FINISH_ROTATE_LEFT;
                }
            break;
        }
    }

    void updateRotateFinalState(double remainingAngle, bool blackSensors[]) {
        bool isFrontLeftBlack = blackSensors[0];
        bool isFrontCenterBlack = blackSensors[1];
        bool isFrontRightBlack = blackSensors[2];

        if (isFrontCenterBlack && (isFrontLeftBlack || isFrontRightBlack)) {
            logs->concat(String(isFrontLeftBlack) + " " + String(isFrontCenterBlack) + " " + String(isFrontRightBlack) + "\n");
            return;
        }

        // Update the ending rotateState, this will be helpful to complete moving
        // and compensating this error while moving, if the next action is move
        if (rotateState == ROTATE_STATE::FINISH_ROTATE_LEFT) {
            if (remainingAngle < 10) {
                rotateState = ROTATE_STATE::FAILED_ROTATE_LEFT_OFFLINE_LEFT;
            } else {
                rotateState = ROTATE_STATE::FAILED_ROTATE_LEFT_OFFLINE_RIGHT;
                logs->concat("Error in excess rotation evaluation!\n");
            }
        } else if (rotateState == ROTATE_STATE::ROTATE_LEFT) {
            if (remainingAngle < 10) {
                rotateState = ROTATE_STATE::FAILED_ROTATE_LEFT_OFFLINE_LEFT;
                logs->concat("Error in excess rotation evaluation!\n");
            } else {
                rotateState = ROTATE_STATE::FAILED_ROTATE_LEFT_OFFLINE_RIGHT;
            }
        } else if (rotateState == ROTATE_STATE::FINISH_ROTATE_RIGHT) {
            if (remainingAngle < 10) {
                rotateState = ROTATE_STATE::FAILED_ROTATE_RIGHT_OFFLINE_RIGHT;
            } else {
                rotateState = ROTATE_STATE::FAILED_ROTATE_RIGHT_OFFLINE_LEFT;
                logs->concat("Error in excess rotation evaluation!\n");
            }
        } else if (rotateState == ROTATE_STATE::ROTATE_RIGHT) {
            if (remainingAngle < 10) {
                rotateState = ROTATE_STATE::FAILED_ROTATE_RIGHT_OFFLINE_RIGHT;
                logs->concat("Error in excess rotation evaluation!\n");
            } else {
                rotateState = ROTATE_STATE::FAILED_ROTATE_RIGHT_OFFLINE_LEFT;
            }
        }

        logs->concat("RemainingAngle: " + String(remainingAngle) + "\n");
        logs->concat("Stopping state: " + String((int) rotateState) + "\n");
    }

    void getMotorsRotateSpeeds(double& leftSpeed, double& rightSpeed) {
        switch (rotateState) {
            case ROTATE_STATE::PREPARE_ROTATE_RIGHT:
            case ROTATE_STATE::PRE_ROTATE_RIGHT:
            case ROTATE_STATE::ROTATE_RIGHT:
            case ROTATE_STATE::POST_ROTATE_RIGHT:
            case ROTATE_STATE::FINISH_ROTATE_RIGHT:
                rightSpeed = -MOTORS_ROTATION_SPEED;
                leftSpeed = MOTORS_ROTATION_SPEED;
            break;

            case ROTATE_STATE::PREPARE_ROTATE_LEFT:
            case ROTATE_STATE::PRE_ROTATE_LEFT:
            case ROTATE_STATE::ROTATE_LEFT:
            case ROTATE_STATE::POST_ROTATE_LEFT:
            case ROTATE_STATE::FINISH_ROTATE_LEFT:
                rightSpeed = MOTORS_ROTATION_SPEED;
                leftSpeed = -MOTORS_ROTATION_SPEED;
            break;
        }

        double leftDistance = leftMotorController.getTotalDistance();
        double rightDistance = rightMotorController.getTotalDistance();

        if (stopLeftMotor && !(rightDistance > leftDistance && stopRightMotor && remainingAngle >= 25)) {
            leftSpeed = 0;
        }

        if (stopRightMotor && !(rightDistance < leftDistance && stopLeftMotor && remainingAngle >= 25)) {
            rightSpeed = 0;
        }
    }

    bool isRotationDone(double remainingAngle, bool blackSensors[]) {
        bool isFrontLeftBlack = blackSensors[0];
        bool isFrontRightBlack = blackSensors[2];

        bool result = false;

        result |= (remainingAngle <= EXCESS_ANGLES_LIMIT);
        result |= (stopLeftMotor && stopRightMotor && remainingAngle < 25);

        return result;
    }

    // ====================
    // Retreat handling functions
    void retreat(double obstacleDistance, double angle, bool blackSensors[]) {
        // Emergency braking
        if ((retreatState == RETREAT_STATE::POST_RETREAT_MOVE ||
            retreatState == RETREAT_STATE::POST_RETREAT_ALIGNMENT)) {
            if (obstacleDistance < MIN_DISTANCE) {
                if (!monitorBlockage) {
                    monitorBlockage = true;
                    blockageTime = millis();
                } else if (millis() - blockageTime > 20) {
                    logs->concat("Error: obstacle at: " + String(obstacleDistance) + "\n");

                    *blocked = true;

                    pause();

                    return;
                }
            } else {
                monitorBlockage = false;
                blockageTime = 0;
            }
        }

        // Update the remaining angle
        remainingAngle -= Utils::anglesSmallDifference(angle, previousAngle);
        previousAngle = angle;

        // Update remaining distances
        double leftDistance = leftMotorController.getTotalDistance();
        double rightDistance = rightMotorController.getTotalDistance();

        double remainingDistance = postRetreatDistance - (leftDistance + rightDistance) / 2.0;

        // FSM transitions
        updateRetreatState(angle, blackSensors, remainingDistance, remainingAngle);

        // Get motors new speeds
        double leftSpeed, rightSpeed;
        getMotorsRetreatSpeeds(angle, remainingAngle, leftSpeed, rightSpeed);

        // Update motors controllers
        leftMotorController.setSpeed(leftSpeed);
        rightMotorController.setSpeed(rightSpeed);

        leftMotorController.update();
        rightMotorController.update();

        // Compensate back black sensors errors
        double diff = Utils::mapAngle(angle - referenceAngle);

        // if (stopLeftMotor && !stopRightMotor && diff < -10 && remainingDistance < STEP / 5) {
        //     stopRightMotor = true;
        //     logs->concat("Force stopping right motor, diff: " + String(diff) + "\n");

        //     moveState = MOVE_STATE::OFFLINE_RIGHT;
        // }

        // if (!stopLeftMotor && stopRightMotor && diff > 10 && remainingDistance < STEP / 5) {
        //     stopLeftMotor = true;
        //     logs->concat("Force stopping left motor, diff: " + String(diff) + "\n");

        //     moveState = MOVE_STATE::OFFLINE_LEFT;
        // }

        // Check if arrived
        if (stopLeftMotor && stopRightMotor) {
            done();
        }

        if (millis() - time > 200 || prevIsFrontLeftBlack != blackSensors[0] ||
            prevIsFrontCenterBlack != blackSensors[1] ||
            prevIsFrontRightBlack != blackSensors[2] ||
            prevIsBackLeftBlack != blackSensors[3] ||
            prevIsBackRightBlack != blackSensors[4]) {

            time = millis();

            prevIsFrontLeftBlack = blackSensors[0];
            prevIsFrontCenterBlack = blackSensors[1];
            prevIsFrontRightBlack = blackSensors[2];
            prevIsBackLeftBlack = blackSensors[3];
            prevIsBackRightBlack = blackSensors[4];


            logs->concat("Rs: " + String((int) retreatState) + "\n");
            logs->concat("Ms: " + String((int) moveState) + "\n");
            logs->concat("angle: " + String(angle) + "\n");
            logs->concat("diff: " + String(Utils::mapAngle(angle - referenceAngle)) + "\n");
            logs->concat("prd: " + String(postRetreatDistance) + "\n");
            logs->concat("refA: " + String(referenceAngle) + "\n");
            logs->concat("red: " + String(remainingDistance) + "\n");
            logs->concat("ld: " + String(leftDistance) + "\n");
            logs->concat("rd: " + String(rightDistance) + "\n");
            logs->concat("lspd: " + String(leftSpeed) + "\n");
            logs->concat("rspd: " + String(rightSpeed) + "\n");
            logs->concat("lstp: " + String(stopLeftMotor) + "\n");
            logs->concat("rstp: " + String(stopRightMotor) + "\n");
            logs->concat("rA: " + String(remainingAngle) + "\n");
            logs->concat("blks: " + String(blackSensors[0]) + " " +
                                    String(blackSensors[1]) + " " +
                                    String(blackSensors[2]) + " " +
                                    String(blackSensors[3]) + " " +
                                    String(blackSensors[4]));
            logs->concat("\n\n\n");
        }
    }

    void updateRetreatState(double angle, bool blackSensors[], double remainingDistance, double remainingAngle) {
        bool isFrontLeftBlack = blackSensors[0];
        bool isFrontCenterBlack = blackSensors[1];
        bool isFrontRightBlack = blackSensors[2];
        bool isBackLeftBlack = blackSensors[3];
        bool isBackRightBlack = blackSensors[4];

        switch (retreatState) {
            case RETREAT_STATE::RETREAT:
                if (remainingAngle <= 15) {
                    retreatState = RETREAT_STATE::HOLD;

                    holdStartTime = millis();
                }
            break;

            case RETREAT_STATE::HOLD:
                if (millis() - holdStartTime > HOLD_STATE_PERIOD) {
                    if (remainingDistance < STEP / 3) {
                        retreatState = RETREAT_STATE::POST_RETREAT_ALIGNMENT;
                    } else if (isFrontLeftBlack && isFrontCenterBlack && isFrontRightBlack) {
                        retreatState = RETREAT_STATE::POST_RETREAT_ALIGNMENT;
                    } else if (isFrontLeftBlack || isFrontCenterBlack || isFrontRightBlack) {
                        retreatState = RETREAT_STATE::POST_RETREAT_MOVE;
                    } else {
                        retreatState = RETREAT_STATE::POST_RETREAT_ALIGNMENT;
                    }

                    // Reset the motors after rotation
                    leftMotorController.reset();
                    rightMotorController.reset();
                }
            break;

            case RETREAT_STATE::POST_RETREAT_MOVE:
                if (isFrontLeftBlack && isFrontCenterBlack && isFrontRightBlack) {
                    retreatState = RETREAT_STATE::POST_RETREAT_ALIGNMENT;
                }
            break;

            case RETREAT_STATE::POST_RETREAT_ALIGNMENT:
                if ((isFrontLeftBlack || isFrontCenterBlack || isFrontRightBlack) &&
                    !(isFrontLeftBlack && isFrontCenterBlack && isFrontRightBlack) &&
                    remainingDistance > STEP / 4) {
                    retreatState = RETREAT_STATE::POST_RETREAT_MOVE;
                }
            break;
        }

        if (retreatState != RETREAT_STATE::RETREAT) {
            // Distances
            double leftDistance = leftMotorController.getTotalDistance();
            double rightDistance = rightMotorController.getTotalDistance();

            // Stopping after moving the postRetreatDistance
            bool leftDistanceReturned = (postRetreatDistance - leftDistance) <= postRetreatDistance / 2;
            bool rightDistanceReturned = (postRetreatDistance - rightDistance) <= postRetreatDistance / 2;

            stopLeftMotor |= (isBackLeftBlack && leftDistanceReturned);
            stopRightMotor |= (isBackRightBlack && rightDistanceReturned);
        }

        if (retreatState == RETREAT_STATE::POST_RETREAT_MOVE) {
            updateMoveState(angle, blackSensors, remainingDistance, 10);
        }
    }

    void getMotorsRetreatSpeeds(double angle, double remainingAngle, double& leftSpeed, double& rightSpeed) {
        switch (retreatState) {
            case RETREAT_STATE::RETREAT:
                rightSpeed = -MOTORS_ROTATION_SPEED;
                leftSpeed = MOTORS_ROTATION_SPEED;
            break;

            case RETREAT_STATE::HOLD:
                rightSpeed = 0;
                leftSpeed = 0;
            break;

            case RETREAT_STATE::POST_RETREAT_MOVE:
                getMotorsMoveSpeeds(angle, leftSpeed, rightSpeed);
            break;

            case RETREAT_STATE::POST_RETREAT_ALIGNMENT:
                getMotorsAlignmentSpeeds(angle, leftSpeed, rightSpeed);
            break;
        }

        if (stopLeftMotor) {
            leftSpeed = 0;
        }

        if (stopRightMotor) {
            rightSpeed = 0;
        }

        leftSpeed /= 2;
        rightSpeed /= 2;
    }

    // ====================
    // Helper functions
};