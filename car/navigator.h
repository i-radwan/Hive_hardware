#pragma once

#include "communicator.h"
#include "sensors/encoder.h"
#include "utils/utils.h"
#include "utils/constants.h"

class Navigator {
public:

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

    void navigate(double currentAngle) {
        if (state == MOVE)
            move(currentAngle);
        else if (state == ALIGN || state == ROTATE)
            rotate(currentAngle);
    }
    
    void stop() {
        state = IDLE;
        avgAngle = 0;
        angelsCnt = 0;

        adjustMotors(PWMRANGE, PWMRANGE, LOW, LOW, LOW, LOW);
    }
    
    void forward(double currentAngle) {
        if (state == INIT)
            init(currentAngle);

        time = millis();
        pidI = 0;
        state = MOVE;
        avgAngle = 0;
        angelsCnt = 0;

        distance += STEP;
    }

    void backward(double currentAngle) {
        if (state == INIT)
            init(currentAngle);

        // time = millis();
        // pidI = 0;
    }

    void left(double currentAngle) {
        if (state == INIT)
            init(currentAngle);

        time = millis();
        pidI = 0;
        state = ROTATE;
        avgAngle = 0;
        angelsCnt = 0;
        
        angle += -90;

        if (angle < -180) {
            angle += 360;
        }
    }

    void right(double currentAngle) {
        if (state == INIT)
            init(currentAngle);

        time = millis();
        pidI = 0;
        state = ROTATE;
        avgAngle = 0;
        angelsCnt = 0;
        
        angle += 90;

        if (angle > 180) {
            angle -= 360;
        }
    }

private:
    Communicator* com;
    PCF857x* pcf1;
    Encoder* len;
    Encoder* ren;

    STATE state = INIT;

    double angle = 0; // Reference angle
    double distance = 0; // Remaining distance

    double time = 0;
    double prevDiff = 0;
    double pid = 0, pidP = 0, pidI = 0, pidD = 0;

    double avgAngle = 0;
    int angelsCnt = 0;

    void init(double currentAngle) {
        angle = currentAngle;

        state = IDLE;
    }

    void move(double currentAngle) {
        // com->send(" ");
        //
        // Angle
        //
        double diff = currentAngle - angle;

        if (diff > 180) {
            diff -= 360;
        } else if (diff < -180) {
            diff += 360;
        }

        avgAngle += currentAngle;
        angelsCnt++;

        //
        // Timer
        //
        double current = millis();

        if (current - time < MOTORS_ADJUST_DELTA)
            return;

        double elapsedTime = (current - time) / 1000.0;
        time = current;

        //
        // Distance
        //
        unsigned long leftTicks, rightTicks;
        
        noInterrupts();
        len->readAndReset(leftTicks);
        ren->readAndReset(rightTicks);
        interrupts();

        double leftDistance = leftTicks / DISK_SLOTS * WHEEL_DIAMETER * M_PI;
        double rightDistance = rightTicks / DISK_SLOTS * WHEEL_DIAMETER * M_PI;
    
        double currentDistance = min(leftDistance, rightDistance);

        distance -= currentDistance;

        angelsCnt = avgAngle = 0;
        
        if (distance <= 0) {
            state = ALIGN;

            return;
        }

        // com->send(String(leftDistance - rightDistance) + " - " + String(leftTicks) + " - " + String(rightTicks) + " - " + String(leftDistance) + " - " + String(rightDistance));

        //
        // PID
        //
        pidP = KP * diff;
        pidI += (diff < I_LIMIT && diff > -I_LIMIT) ? (KI * diff) : 0;
        pidD = KD * ((diff - prevDiff) / elapsedTime);
        prevDiff = diff;

        pid = constrain(pidP + pidI + pidD, -PWMRANGE, PWMRANGE);

        double leftPWM = constrain(MOTORS_INIT_PWM - pid, -PWMRANGE, PWMRANGE);
        double rightPWM = constrain(MOTORS_INIT_PWM + pid, -PWMRANGE, PWMRANGE);

        // Set directions
        int lDir1 = HIGH;
        int lDir2 = LOW;
        int rDir1 = HIGH;
        int rDir2 = LOW;
        
        // com->send("Diff: " + String(diff) + " currentAngle: " + String(currentAngle) + " angle: " + String(angle) + " distance: " + String(distance) + " pid: " + String(pid) + " leftPWM: " + String(leftPWM) + " rightPWM: " + String(rightPWM));

        adjustMotors(leftPWM * LF, rightPWM * RF, lDir1, lDir2, rDir1, rDir2);
    }

    void rotate(double currentAngle) {
        // com->send(" ");
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
        // Timer
        //
        double current = millis();

        if (current - time < MOTORS_ADJUST_DELTA)
            return;

        double elapsedTime = (current - time) / 1000.0;
        time = current;

        //
        // PID
        //
        pidP = KP * diff / 5;
        pidI += (diff < I_LIMIT && diff > -I_LIMIT) ? (KI * diff) : 0;
        pidD = KD * ((diff - prevDiff) / elapsedTime);
        prevDiff = diff;

        pid = constrain(pidP + pidI + pidD, -PWMRANGE, PWMRANGE);

        // com->send("Diff: " + String(diff) + " currentAngle: " + String(currentAngle) + " angle: " + String(angle) + " distance: " + String(distance) + " pid: " + String(pid));

        if (abs(diff) < 4 || abs(pid) < 10) {
            adjustMotors(PWMRANGE, PWMRANGE, LOW, LOW, LOW, LOW);
            
            // Remove the ticks to not consider them as distance when moving forward
            noInterrupts();
            len->reset();
            ren->reset();
            interrupts();
            
            return;
        }

        double leftPWM, rightPWM;

        if (diff < 0) { // Turn right
            leftPWM = constrain(MOTORS_INIT_PWM - pid, 0, PWMRANGE / 4);
            rightPWM = constrain(-MOTORS_INIT_PWM + pid, -PWMRANGE / 4, 0);
        } else { // Turn left
            leftPWM = constrain(-MOTORS_INIT_PWM - pid, -PWMRANGE / 4, 0);
            rightPWM = constrain(MOTORS_INIT_PWM + pid, 0, PWMRANGE / 4);
        }

        // Set directions
        int lDir1 = HIGH;
        int lDir2 = LOW;
        int rDir1 = HIGH;
        int rDir2 = LOW;

         // -ve values mean the other direction
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