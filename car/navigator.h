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

    void setup(PCF857x* pcf1, Encoder* len, Encoder* ren, String* logsPtr) {
        // Configure motors controllers
        leftMotorController.en = len;
        rightMotorController.en = ren;

        leftMotorController.pcf = pcf1;
        rightMotorController.pcf = pcf1;

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
            retreat(angle, blackSensors);
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

        motorsLeftBlackLines = false;
        stopLeftMotor = false;
        stopRightMotor = false;
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

    void move(double angle) {
        if (executionState.state == EXECUTION_STATE::IDLE) {
            prepare();

            moveState = MOVE_STATE::STRAIGHT;

            // Recover from an incomplete rotation
            if (action == ACTION::ROTATE_LEFT) {
                if (rotateState == ROTATE_STATE::FAILED_ROTATE_LEFT_OFFLINE_LEFT) {
                    moveState = MOVE_STATE::DRIFTING_LEFT;
                } else if (rotateState == ROTATE_STATE::FAILED_ROTATE_LEFT_OFFLINE_RIGHT) {
                    moveState = MOVE_STATE::DRIFTING_RIGHT;
                }
            } else if (action == ACTION::ROTATE_RIGHT) {
                if (rotateState == ROTATE_STATE::FAILED_ROTATE_RIGHT_OFFLINE_LEFT) {
                    moveState = MOVE_STATE::DRIFTING_LEFT;
                } else if (rotateState == ROTATE_STATE::FAILED_ROTATE_RIGHT_OFFLINE_RIGHT) {
                    moveState = MOVE_STATE::DRIFTING_RIGHT;
                }
            }

            action = ACTION::MOVE;
            executionState.state = EXECUTION_STATE::ONGOING;

            addStraightAngle(angle);

            leftMotorController.setSpeedIncrementStep(MOTORS_MOVE_SPEED_INCREMENT);
            rightMotorController.setSpeedIncrementStep(MOTORS_MOVE_SPEED_INCREMENT);

            logs->concat("Start moving\n");
        } else if (executionState.state == EXECUTION_STATE::PAUSE && action == ACTION::MOVE) {
            executionState.state = EXECUTION_STATE::ONGOING;

            logs->concat("Resume moving\n");
        }
    }

    void rotateRight(double angle) {
        if (executionState.state == EXECUTION_STATE::IDLE) {
            prepare();

            action = ACTION::ROTATE_RIGHT;
            rotateState = ROTATE_STATE::PREPARE_ROTATE_RIGHT;
            executionState.state = EXECUTION_STATE::ONGOING;

            remainingAngle = 90;
            previousAngle = angle;

            straightAnglesSin = 0;
            straightAnglesCos = 0;
            straightAnglesCnt = 0;

            leftMotorController.setSpeedIncrementStep(MOTORS_ROTATE_SPEED_INCREMENT);
            rightMotorController.setSpeedIncrementStep(MOTORS_ROTATE_SPEED_INCREMENT);

            logs->concat("Start rotating right\n");
        } else if (executionState.state == EXECUTION_STATE::PAUSE && action == ACTION::ROTATE_RIGHT) {
            executionState.state = EXECUTION_STATE::ONGOING;

            logs->concat("Resume rotating right\n");
        }
    }

    void rotateLeft(double angle) {
        if (executionState.state == EXECUTION_STATE::IDLE) {
            prepare();

            action = ACTION::ROTATE_LEFT;
            rotateState = ROTATE_STATE::PREPARE_ROTATE_LEFT;
            executionState.state = EXECUTION_STATE::ONGOING;

            remainingAngle = 90;
            previousAngle = angle;

            straightAnglesSin = 0;
            straightAnglesCos = 0;
            straightAnglesCnt = 0;

            leftMotorController.setSpeedIncrementStep(MOTORS_ROTATE_SPEED_INCREMENT);
            rightMotorController.setSpeedIncrementStep(MOTORS_ROTATE_SPEED_INCREMENT);

            logs->concat("Start rotating left\n");
        } else if (executionState.state == EXECUTION_STATE::PAUSE && action == ACTION::ROTATE_LEFT) {
            executionState.state = EXECUTION_STATE::ONGOING;

            logs->concat("Resume rotating left\n");
        }
    }

    void retreat(double angle) {
        if (executionState.state == EXECUTION_STATE::PAUSE && action == ACTION::MOVE) {
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
            postRetreatAngle = (getStraightAngle() + 180) % 360;

            straightAnglesSin = 0;
            straightAnglesCos = 0;
            straightAnglesCnt = 0;

            if (moveState == MOVE_STATE::STRAIGHT) {
                remainingAngle = Utils::anglesAverageDifference(angle, postRetreatAngle);
            } else if (moveState == MOVE_STATE::STRAIGHT_RIGHT ||
                       moveState == MOVE_STATE::DRIFTING_RIGHT ||
                       moveState == MOVE_STATE::OFFLINE_RIGHT) {
                remainingAngle = Utils::anglesSmallDifference(angle, postRetreatAngle);
            } else if (moveState == MOVE_STATE::STRAIGHT_LEFT ||
                       moveState == MOVE_STATE::DRIFTING_LEFT ||
                       moveState == MOVE_STATE::OFFLINE_LEFT) {
                remainingAngle = Utils::anglesLargeDifference(angle, postRetreatAngle);
            }

            postRetreatDistance = movedDistance;

            leftMotorController.setSpeedIncrementStep(MOTORS_MOVE_SPEED_INCREMENT / 2);
            rightMotorController.setSpeedIncrementStep(MOTORS_MOVE_SPEED_INCREMENT / 2);

            logs->concat("Start retreating\n");
            logs->concat("MS: " + String((int) moveState) + " Current angle: " + String(angle) + " pra: " + postRetreatAngle + " ra: " + String(remainingAngle) + "\n");
        } else if (executionState.state == EXECUTION_STATE::PAUSE && action == ACTION::RETREAT) {
            executionState.state = EXECUTION_STATE::ONGOING;

            logs->concat("Resume reterating\n");
        }
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

    bool motorsLeftBlackLines = false;
    bool stopLeftMotor = false;
    bool stopRightMotor = false;

    double straightAnglesCos = 0;
    double straightAnglesSin = 0;
    int straightAnglesCnt = 0;

    double remainingAngle;
    double previousAngle;

    double postRetreatAngle;
    double postRetreatDistance;

    double time = millis();

    // ====================
    // Execution state updating function

    void init() {
        executionState.state = EXECUTION_STATE::IDLE;

        leftMotorController.brake();
        rightMotorController.brake();

        logs->concat("Init\n");
    }

    void pause() {
        executionState.state = EXECUTION_STATE::PAUSE;

        leftMotorController.brake();
        rightMotorController.brake();

        logs->concat("Pausing\n");
    }

    void done() {
        executionState.state = EXECUTION_STATE::FINISHED;

        leftMotorController.brake();
        rightMotorController.brake();

        logs->concat("Done\n");
    }

    void halt(EXECUTION_ERROR error) {
        executionState.state = EXECUTION_STATE::ERROR;
        executionState.error = error;

        leftMotorController.brake();
        rightMotorController.brake();

        logs->concat("Halting due: " + String((int) error) + "\n");
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
            pause();

            return;
        }

        // Update remaining distances
        double leftDistance = leftMotorController.getTotalDistance();
        double rightDistance = rightMotorController.getTotalDistance();

        double remainingDistance = STEP - (leftDistance + rightDistance) / 2.0;
        double minimumDistanceToNode = STEP / 2.0;

        if (remainingDistance < -EXCESS_DISTANCE_LIMIT) {
            halt(EXECUTION_ERROR::EXCEEDED_ALLOWED_DISTANCE);

            return;
        }

        // FSM transitions
        updateMoveState(angle, blackSensors, remainingDistance, minimumDistanceToNode);

        // Store angles to prepare for moving using angles when reaching the next node,
        // where all sensors see black
        if (moveState == MOVE_STATE::STRAIGHT) {
            addStraightAngle(angle);
        }

        // Get motors new speeds
        double leftSpeed, rightSpeed;
        getMotorsMoveSpeeds(angle, getStraightAngle(), leftSpeed, rightSpeed);

        // Update motors controllers
        leftMotorController.setSpeed(leftSpeed);
        rightMotorController.setSpeed(rightSpeed);

        leftMotorController.update();
        rightMotorController.update();

        // Compensate back black sensors errors
        double diff = Utils::mapAngle(angle - getStraightAngle());

        if (stopLeftMotor && !stopRightMotor && diff < -10 && remainingDistance < STEP / 5) {
            stopRightMotor = true;
            logs->concat("Force stopping right motor, diff: " + String(diff) + "\n");
        }

        if (!stopLeftMotor && stopRightMotor && diff > 10 && remainingDistance < STEP / 5) {
            stopLeftMotor = true;
            logs->concat("Force stopping left motor, diff: " + String(diff) + "\n");
        }

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
            logs->concat("RefA: " + String(getStraightAngle()) + "\n");
            logs->concat("lspd: " + String(leftMotorController.getSpeed()) + "\n");
            logs->concat("rspd: " + String(rightMotorController.getSpeed()) + "\n");
            logs->concat("rd: " + String(remainingDistance) + "\n");
            logs->concat("mdtn: " + String(minimumDistanceToNode) + "\n");
            logs->concat("mlbl: " + String(motorsLeftBlackLines) + "\n");
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

        if (!isBackLeftBlack && !isBackRightBlack) {
            motorsLeftBlackLines = true;
        }

        // Stopping at nodes
        if (remainingDistance < minimumDistanceToNode && motorsLeftBlackLines) {
            stopLeftMotor = stopLeftMotor || isBackLeftBlack;
            stopRightMotor = stopRightMotor || isBackRightBlack;
        }


        if (moveState == MOVE_STATE::ALIGNMENT && remainingDistance <= STEP / 4) {
            return;
        }

        // FSM transitions
        if (isFrontCenterBlack && isFrontLeftBlack && isFrontRightBlack) {
            moveState = MOVE_STATE::STRAIGHT;

            if (remainingDistance <= STEP / 4) {
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
            } else if (moveState == MOVE_STATE::STRAIGHT || moveState == MOVE_STATE::HOLD) {
                double straightAngle = getStraightAngle();
                double diff = Utils::mapAngle(angle - straightAngle);

                logs->concat("ERROR: " + String(straightAngle) + " " + String(diff) + "\n");

                if (abs(diff) < EPS) {
                    moveState = MOVE_STATE::HOLD;
                } else if (diff < 0) {
                    moveState = MOVE_STATE::OFFLINE_LEFT;
                } else {
                    moveState = MOVE_STATE::OFFLINE_RIGHT;
                }
            }
        }
    }

    void getMotorsMoveSpeeds(double angle, double referenceAngle, double& leftSpeed, double& rightSpeed) {
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
                rightSpeed = MOTORS_SPEED * 0.5;
            break;

            case MOVE_STATE::STRAIGHT_RIGHT:
                leftSpeed = MOTORS_SPEED * 0.5;
                rightSpeed = MOTORS_SPEED;
            break;

            case MOVE_STATE::DRIFTING_LEFT:
                leftSpeed = MOTORS_SPEED * 0.6;
                rightSpeed = MOTORS_SPEED * 0.2;
            break;

            case MOVE_STATE::DRIFTING_RIGHT:
                leftSpeed = MOTORS_SPEED * 0.2;
                rightSpeed = MOTORS_SPEED * 0.6;
            break;

            case MOVE_STATE::OFFLINE_LEFT:
                leftSpeed = MOTORS_SPEED * 0.4;
                rightSpeed = 0;
            break;

            case MOVE_STATE::OFFLINE_RIGHT:
                leftSpeed = 0;
                rightSpeed = MOTORS_SPEED * 0.4;
            break;

            case MOVE_STATE::ALIGNMENT:
                getMotorsAlignmentSpeeds(angle, referenceAngle, leftSpeed, rightSpeed);
            break;
        }

        // Stopping at nodes
        if (stopLeftMotor) {
            leftSpeed = 0;
        }

        if (stopRightMotor) {
            rightSpeed = 0;
        }
    }

    void getMotorsAlignmentSpeeds(double angle, double referenceAngle, double& leftSpeed, double& rightSpeed) {
        double diff = Utils::mapAngle(angle - referenceAngle);

        if (diff < -10) { // (-oo, -10)
            leftSpeed = MOTORS_SPEED * 0.5;
            rightSpeed = MOTORS_SPEED * 0.3;
        } else if (diff < -3) { // [-10, -3)
            leftSpeed = MOTORS_SPEED * 0.8;
            rightSpeed = MOTORS_SPEED * 0.5;
        } else if (diff > 3) { // (3, 10]
            leftSpeed = MOTORS_SPEED * 0.5;
            rightSpeed = MOTORS_SPEED * 0.8;
        } else if (diff > 10) { // (10, oo)
            leftSpeed = MOTORS_SPEED * 0.3;
            rightSpeed = MOTORS_SPEED * 0.5;
        } else { // [-3, 3]
            leftSpeed = MOTORS_SPEED * 0.8;
            rightSpeed = MOTORS_SPEED * 0.8;
        }

        // Stopping at nodes
        if (stopLeftMotor) {
            leftSpeed = 0;
        }

        if (stopRightMotor) {
            rightSpeed = 0;
        }
    }

    // ====================
    // Rotation handling functions

    void rotate(double angle, bool blackSensors[]) {
        // Update the remaining angle
        remainingAngle -= Utils::anglesSmallDifference(angle, previousAngle);
        previousAngle = angle;

        // FSM transitions
        updateRotateState(blackSensors, remainingAngle);

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
        if (isRotationDone(blackSensors, remainingAngle)) {
            done();

            updateRotateFinalState(blackSensors, remainingAngle);
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

            logs->concat("RS: " + String((int) rotateState) + "\n");
            logs->concat("a: " + String(angle) + "\n");
            logs->concat("ra: " + String(remainingAngle) + "\n");
            logs->concat("ld: " + String(leftMotorController.getTotalDistance()) + "\n");
            logs->concat("rd: " + String(rightMotorController.getTotalDistance()) + "\n");
            logs->concat("mlbl: " + String(motorsLeftBlackLines) + "\n");
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

    void updateRotateState(bool blackSensors[], double remainingAngle) {
        bool isFrontLeftBlack = blackSensors[0];
        bool isFrontCenterBlack = blackSensors[1];
        bool isFrontRightBlack = blackSensors[2];
        bool isBackLeftBlack = blackSensors[3];
        bool isBackRightBlack = blackSensors[4];

        // Distances
        double leftDistance = leftMotorController.getTotalDistance();
        double rightDistance = rightMotorController.getTotalDistance();

        if ((!isBackLeftBlack && leftDistance > 30) && (!isBackRightBlack && rightDistance > 30)) {
            motorsLeftBlackLines = true;
        }

        // Stopping after rotation or
        // when one wheel spins much more than the expected rotation distance
        if (motorsLeftBlackLines) {
            stopLeftMotor |= isBackLeftBlack || leftDistance > 120;
            stopRightMotor |= isBackRightBlack || rightDistance > 125;
        }

        switch (rotateState) {
            case ROTATE_STATE::PREPARE_ROTATE_RIGHT:
                if (!isFrontRightBlack) {
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

    void updateRotateFinalState(bool blackSensors[], double remainingAngle) {
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
            rotateState = ROTATE_STATE::FAILED_ROTATE_LEFT_OFFLINE_LEFT;
        } else if (rotateState == ROTATE_STATE::ROTATE_LEFT) {
            rotateState = ROTATE_STATE::FAILED_ROTATE_LEFT_OFFLINE_RIGHT;
        } else if (rotateState == ROTATE_STATE::FINISH_ROTATE_RIGHT) {
            rotateState = ROTATE_STATE::FAILED_ROTATE_RIGHT_OFFLINE_RIGHT;
        } else if (rotateState == ROTATE_STATE::ROTATE_RIGHT) {
            rotateState = ROTATE_STATE::FAILED_ROTATE_RIGHT_OFFLINE_LEFT;
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

        if (stopLeftMotor) {
            leftSpeed = 0;
        }

        if (stopRightMotor) {
            rightSpeed = 0;
        }
    }

    bool isRotationDone(bool blackSensors[], double remainingAngle) {
        bool isFrontLeftBlack = blackSensors[0];
        bool isFrontRightBlack = blackSensors[2];

        bool result = false;

        result |= (remainingAngle <= -EXCESS_ANGLES_LIMIT);
        result |= (stopLeftMotor && stopRightMotor);

        if (result) {
            logs->concat("\nResult::\n");
            logs->concat("rA" + String(remainingAngle) + "\n");
            logs->concat("slm" + String(stopLeftMotor) + "\n");
            logs->concat("srm" + String(stopRightMotor) + "\n");
        }

        return result;
    }

    // ====================
    // Retreat handling functions
    void retreat(double angle, bool blackSensors[]) {
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
        getMotorsRetreatSpeeds(angle, postRetreatAngle, remainingAngle, leftSpeed, rightSpeed);

        // Update motors controllers
        leftMotorController.setSpeed(leftSpeed);
        rightMotorController.setSpeed(rightSpeed);

        leftMotorController.update();
        rightMotorController.update();

        // Check if arrived
        if (stopLeftMotor && stopRightMotor) {
            done();
        }

        if (millis() - time > 100) {
            time = millis();

            logs->concat("Rs: " + String((int) retreatState) + "\n");
            logs->concat("Ms: " + String((int) moveState) + "\n");
            logs->concat("prd: " + String(postRetreatDistance) + "\n");
            logs->concat("red: " + String(remainingDistance) + "\n");
            logs->concat("ld: " + String(leftDistance) + "\n");
            logs->concat("rd: " + String(rightDistance) + "\n");
            logs->concat("lspd: " + String(leftSpeed) + "\n");
            logs->concat("rspd: " + String(rightSpeed) + "\n");
            logs->concat("lstp: " + String(stopLeftMotor) + "\n");
            logs->concat("rstp: " + String(stopRightMotor) + "\n");
            logs->concat("rA: " + String(remainingAngle) + "\n");
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
                if (remainingAngle <= 10) {
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
            bool leftDistanceReturned = (postRetreatDistance - leftDistance) <= 10;
            bool rightDistanceReturned = (postRetreatDistance - rightDistance) <= 10;

            stopLeftMotor = stopLeftMotor || (isBackLeftBlack && leftDistanceReturned);
            stopRightMotor = stopRightMotor || (isBackRightBlack && rightDistanceReturned);
        }

        if (retreatState == RETREAT_STATE::POST_RETREAT_MOVE) {
            updateMoveState(angle, blackSensors, remainingDistance, 10);
        }
    }

    void getMotorsRetreatSpeeds(double angle, double referenceAngle, double remainingAngle, double& leftSpeed, double& rightSpeed) {
        switch (retreatState) {
            case RETREAT_STATE::RETREAT:
                rightSpeed = -MOTORS_ROTATION_SPEED;
                leftSpeed = MOTORS_ROTATION_SPEED;
            break;

            case RETREAT_STATE::POST_RETREAT_MOVE:
                getMotorsMoveSpeeds(angle, referenceAngle, leftSpeed, rightSpeed);
            break;

            case RETREAT_STATE::POST_RETREAT_ALIGNMENT:
                getMotorsAlignmentSpeeds(angle, referenceAngle, leftSpeed, rightSpeed);
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
    void addStraightAngle(double angle) {
        double angleSin, angleCos;
        Utils::angleResolution(Utils::degreesToRad(angle), angleCos, angleSin);

        straightAnglesCos += angleCos;
        straightAnglesSin += angleSin;

        straightAnglesCnt++;
    }

    int getStraightAngle() {
        double straightAnglesAvgSin  = straightAnglesSin / straightAnglesCnt;
        double straightAnglesAvgCos  = straightAnglesCos / straightAnglesCnt;

        return Utils::radToDegrees(Utils::vecToAngle(straightAnglesAvgCos, straightAnglesAvgSin));
    }
};