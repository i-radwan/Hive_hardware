#pragma once

#include "communicator.h"
#include "motor_controller.h"
#include "sensors/encoder.h"
#include "utils/utils.h"
#include "utils/constants.h"

class Navigator {

public:

    Navigator() : leftMotorController(LEFT_KP, LEFT_KI, LEFT_KD, LEFT_SPED, LEFT_DIR1, LEFT_DIR2),
                  rightMotorController(RIGHT_KP, RIGHT_KI, RIGHT_KD, RIGHT_SPED, RIGHT_DIR1, RIGHT_DIR2) {
    }

    void setup(PCF857x* pcf1, Encoder* len, Encoder* ren, String* logsPtr) {
        // Configure motors controllers
        leftMotorController.en = len;
        rightMotorController.en = ren;

        leftMotorController.pcf = pcf1;
        rightMotorController.pcf = pcf1;

        // Logs
        logs = logsPtr;

        // ToDo: read last checkpoint from the files system to handle car sudden restart.
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

            straightAnglesSum = angle;
            straightAnglesCnt = 1;

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

            remainingAngle = 180;
            previousAngle = angle;
            postRetreatAngle = Utils::mapAngle(angle + 180);
            postRetreatDistance = STEP - movedDistance;

            leftMotorController.setSpeedIncrementStep(MOTORS_MOVE_SPEED_INCREMENT);
            rightMotorController.setSpeedIncrementStep(MOTORS_MOVE_SPEED_INCREMENT);

            logs->concat("Start retreating\n");
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

    double straightAnglesSum = 0;
    int straightAnglesCnt = 0;

    double remainingAngle;
    double previousAngle;

    double postRetreatAngle;
    double postRetreatDistance;

    // ====================
    // Execution state updating function

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
        updateMoveState(blackSensors, remainingDistance, minimumDistanceToNode);

        // Store angles to prepare for moving using angles when reaching the next node,
        // where all sensors see black
        if (moveState == MOVE_STATE::STRAIGHT) { // ToDo
            // straightAnglesSum += angle;
            // straightAnglesCnt++;
        }

        // Get motors new speeds
        double leftSpeed, rightSpeed;
        getMotorsMoveSpeeds(angle, straightAnglesSum / straightAnglesCnt, leftSpeed, rightSpeed);

        // Update motors controllers
        leftMotorController.setSpeed(leftSpeed);
        rightMotorController.setSpeed(rightSpeed);

        leftMotorController.update();
        rightMotorController.update();

        // Check if arrived
        if (stopLeftMotor && stopRightMotor) {
            done();
        }
    }

    void updateMoveState(bool blackSensors[], double remainingDistance, double minimumDistanceToNode) {
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

        // FSM transitions
        if (isFrontCenterBlack && isFrontLeftBlack && isFrontRightBlack) {
            moveState = MOVE_STATE::STRAIGHT;
        } else if (isFrontCenterBlack && isFrontLeftBlack && !isFrontRightBlack) {
            moveState = MOVE_STATE::DRIFTING_RIGHT;
        } else if (isFrontCenterBlack && !isFrontLeftBlack && isFrontRightBlack) {
            moveState = MOVE_STATE::DRIFTING_LEFT;
        } else if (isFrontCenterBlack && !isFrontLeftBlack && !isFrontRightBlack) {
            moveState = MOVE_STATE::STRAIGHT;
        } else if (!isFrontCenterBlack && isFrontLeftBlack && isFrontRightBlack) {
            moveState = MOVE_STATE::STRAIGHT;
        } else if (!isFrontCenterBlack && isFrontLeftBlack && !isFrontRightBlack) {
            moveState = MOVE_STATE::OFFLINE_RIGHT;
        } else if (!isFrontCenterBlack && !isFrontLeftBlack && isFrontRightBlack) {
            moveState = MOVE_STATE::OFFLINE_LEFT;
        } else if (!isFrontCenterBlack && !isFrontLeftBlack && !isFrontRightBlack) {
            if (moveState == MOVE_STATE::DRIFTING_RIGHT) {
                moveState = MOVE_STATE::OFFLINE_RIGHT;
            } else if (moveState == MOVE_STATE::DRIFTING_LEFT) {
                moveState = MOVE_STATE::OFFLINE_LEFT;
            }
        }
    }

    void getMotorsMoveSpeeds(double angle, double referenceAngle, double& leftSpeed, double& rightSpeed) {
        switch (moveState) {
            case MOVE_STATE::STRAIGHT:
                leftSpeed = MOTORS_SPEED;
                rightSpeed = MOTORS_SPEED;
            break;

            case MOVE_STATE::DRIFTING_LEFT:
                leftSpeed = MOTORS_SPEED;
                rightSpeed = MOTORS_SPEED * 0.5;
            break;

            case MOVE_STATE::DRIFTING_RIGHT:
                leftSpeed = MOTORS_SPEED * 0.5;
                rightSpeed = MOTORS_SPEED;
            break;

            case MOVE_STATE::OFFLINE_LEFT:
                leftSpeed = MOTORS_SPEED * 0.5;
                rightSpeed = 0;
            break;

            case MOVE_STATE::OFFLINE_RIGHT:
                leftSpeed = 0;
                rightSpeed = MOTORS_SPEED * 0.5;
            break;

            case MOVE_STATE::ALIGNMENT:
                getMotorsAlignmentSpeeds(angle, referenceAngle, leftSpeed, rightSpeed);
            break;
        }
    }

    void getMotorsAlignmentSpeeds(double angle, double referenceAngle, double& leftSpeed, double& rightSpeed) {
        double diff = Utils::mapAngle(angle - referenceAngle);

        if (diff < -10) { // (-oo, -10)
            leftSpeed = MOTORS_SPEED * 0.5;
            rightSpeed = 0;
        } else if (diff < -3) { // [-10, -3)
            leftSpeed = MOTORS_SPEED * 0.8;
            rightSpeed = MOTORS_SPEED * 0.5;
        } else if (diff > 3) { // (3, 10]
            leftSpeed = MOTORS_SPEED * 0.5;
            rightSpeed = MOTORS_SPEED * 0.6;
        } else if (diff > 10) { // (10, oo)
            leftSpeed = 0;
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
    }

    void updateRotateState(bool blackSensors[], double remainingAngle) {
        bool isFrontLeftBlack = blackSensors[0];
        bool isFrontCenterBlack = blackSensors[1];
        bool isFrontRightBlack = blackSensors[2];
        bool isBackLeftBlack = blackSensors[3];
        bool isBackRightBlack = blackSensors[4];

        if (!isBackLeftBlack && !isBackRightBlack) {
            motorsLeftBlackLines = true;
        }

        // Distances
        double leftDistance = leftMotorController.getTotalDistance();
        double rightDistance = rightMotorController.getTotalDistance();

        // Stopping after rotation or
        // when one wheel spins much more than the expected rotation distance
        if (motorsLeftBlackLines && remainingAngle <= 25) {
            stopLeftMotor = stopLeftMotor || isBackLeftBlack || leftDistance > 185;
            stopRightMotor = stopRightMotor || isBackRightBlack || rightDistance > 185;
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

        if (isFrontCenterBlack || isFrontLeftBlack || isFrontRightBlack) {
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

        logs->concat("Rotation limit exceeded, remainingAngle: " + String(remainingAngle) + "\n");
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
        result |= (stopLeftMotor && stopRightMotor && remainingAngle <= 20);

        if (result) {
            logs->concat(String(remainingAngle <= -EXCESS_ANGLES_LIMIT) + "\n");
            logs->concat(String(stopLeftMotor && stopRightMotor && remainingAngle <= 20) + "\n");
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
        updateRetreatState(blackSensors, remainingDistance, remainingAngle);

        // Get motors new speeds
        double leftSpeed, rightSpeed;
        getMotorsRetreatSpeeds(angle, postRetreatAngle, leftSpeed, rightSpeed);

        // Update motors controllers
        leftMotorController.setSpeed(leftSpeed);
        rightMotorController.setSpeed(rightSpeed);

        leftMotorController.update();
        rightMotorController.update();

        // Check if arrived
        if (stopLeftMotor && stopRightMotor) {
            done();
        }
    }

    void updateRetreatState(bool blackSensors[], double remainingDistance, double remainingAngle) {
        bool isBackLeftBlack = blackSensors[3];
        bool isBackRightBlack = blackSensors[4];

        if (abs(remainingAngle) <= 10) {
            retreatState = RETREAT_STATE::ALIGNMENT;
        }

        if (retreatState == RETREAT_STATE::ALIGNMENT) {
            // Stopping after moving the postRetreatDistance
            bool distanceReturned = (remainingDistance < 0.5 * postRetreatDistance);

            stopLeftMotor = stopLeftMotor || (isBackLeftBlack && distanceReturned);
            stopRightMotor = stopRightMotor || (isBackRightBlack && distanceReturned);
        }
    }

    void getMotorsRetreatSpeeds(double angle, double referenceAngle, double& leftSpeed, double& rightSpeed) {
        switch (retreatState) {
            case RETREAT_STATE::RETREAT:
                rightSpeed = -MOTORS_ROTATION_SPEED;
                leftSpeed = MOTORS_ROTATION_SPEED;
            break;

            case RETREAT_STATE::ALIGNMENT:
                getMotorsAlignmentSpeeds(angle, referenceAngle, leftSpeed, rightSpeed);
            break;
        }

        if (stopLeftMotor) {
            leftSpeed = 0;
        }

        if (stopRightMotor) {
            rightSpeed = 0;
        }
    }
};