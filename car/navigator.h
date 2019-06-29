#pragma once

#include "communicator.h"
#include "motor_controller.h"
#include "sensors/encoder.h"
#include "utils/utils.h"
#include "utils/constants.h"

class Navigator {

public:

    Navigator() : leftMotorController(LEFT_KP, LEFT_KI, LEFT_KD, 0, LEFT_SPED, LEFT_DIR1, LEFT_DIR2),
                  rightMotorController(RIGHT_KP, RIGHT_KI, RIGHT_KD, 0, RIGHT_SPED, RIGHT_DIR1, RIGHT_DIR2) {
    }

    void setup(PCF857x* pcf1, Encoder* len, Encoder* ren, String* logsPtr) {
        // Configure motors controllers
        leftMotorController.en = len;
        rightMotorController.en = ren;

        leftMotorController.pcf = pcf1;
        rightMotorController.pcf = pcf1;

        // Logs
        logs = logsPtr;

        // ToDo: read last checkpoing from the files system to handle car sudden restart.
    }

    void navigate(double obstacleDistance, double angle, bool blackSensors[], ExecutionState& execState) {
        if (executionState.state != EXECUTION_STATE::ONGOING) {
            return;
        }

        if (action == ACTION::MOVE) {
            move(obstacleDistance, angle, blackSensors);
        } else if (action == ACTION::ROTATE_RIGHT || action == ACTION::ROTATE_LEFT) {
            // rotate(angle, blackSensors);
        } else if (action == ACTION::RETREAT) {
            // retreat(angle, blackSensors);
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
    }

    void stop() {
        if (executionState.state == EXECUTION_STATE::IDLE) {
            return;
        }

        pause();

        logs->concat("Stopped\n");
    }

    void move(double angle) {
        prepare();

        if (executionState.state == EXECUTION_STATE::IDLE) {
            action = ACTION::MOVE;
            moveState = MOVE_STATE::STRAIGHT;
            executionState.state = EXECUTION_STATE::ONGOING;

            straightAnglesSum = angle;
            straightAnglesCnt = 1;

            leftMotorController.reset();
            rightMotorController.reset();

            stopLeftMotor = false;
            stopRightMotor = false;

            logs->concat("Start moving\n");
        } else if (executionState.state == EXECUTION_STATE::PAUSE && action == ACTION::MOVE) {
            executionState.state = EXECUTION_STATE::ONGOING;

            logs->concat("Resume moving\n");
        }

        // ToDo: consider moving after rotating left/right
    }

    void rotateRight(double angle) {
        logs->concat("Start rotating right\n");
    }

    void rotateLeft(double angle) {
        logs->concat("Start rotating left\n");
    }

    void retreat(double angle) {
        logs->concat("Start retreating\n");
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

    bool stopLeftMotor = false;
    bool stopRightMotor = false;

    double straightAnglesSum = 0;
    int straightAnglesCnt = 0;

    // ====================
    // Functions

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

    void move(double obstacleDistance, double angle, bool blackSensors[]) {
        // Emergency braking
        if (obstacleDistance < MIN_DISTANCE) {
            pause();

            return;
        }

        // Update distances
        double leftDistance = leftMotorController.getTotalDistance();
        double rightDistance = rightMotorController.getTotalDistance();

        double remainingDistance = STEP - (leftDistance + rightDistance) / 2.0;
        double minimumDistanceToNode = STEP / 2.0;

        if (remainingDistance < -EXCESS_DISTANCE_LIMIT) {
            // halt(EXECUTION_ERROR::EXCEEDED_ALLOWED_DISTANCE);

            // return;
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

        // Stopping at nodes
        if (remainingDistance < minimumDistanceToNode) {
            stopLeftMotor = stopLeftMotor || isBackLeftBlack;
            stopRightMotor = stopRightMotor || isBackRightBlack;
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
};