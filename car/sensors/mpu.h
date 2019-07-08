#pragma once

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

#include "../utils/constants.h"

class MPUSensor {

public:

    void setup(String* logsPtr, bool& failure) {
        logs = logsPtr;

        // Serial.println("Initializing MPU...");
        mpu.initialize();

        // Serial.println("Testing device connections...");
        // Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

        dmpReady = mpu.testConnection();

        // Serial.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();

        // Sensors offsets
        mpu.setXAccelOffset(MPU_ACCEL_X_OFF);
        mpu.setYAccelOffset(MPU_ACCEL_Y_OFF);
        mpu.setZAccelOffset(MPU_ACCEL_Z_OFF);
        mpu.setXGyroOffset(MPU_GYRO_X_OFF);
        mpu.setYGyroOffset(MPU_GYRO_Y_OFF);
        mpu.setZGyroOffset(MPU_GYRO_Z_OFF);

        if (devStatus == 0) {
            // Serial.println("Enabling DMP...");

            mpu.setDMPEnabled(true);

            mpuIntStatus = mpu.getIntStatus();
            packetSize = mpu.dmpGetFIFOPacketSize();

            dmpReady &= true;
        } else {
            // Serial.println("DMP Initialization failed (code " + String(devStatus) + ")");
        }

        if (!dmpReady)
            failure = true;
    }

    bool read(double& y, double& p, double& r) {
        if (!dmpReady) return false;

        double current = millis();

        if (current - lastRead < MPU_REFRESH_RATE) return false;

        lastRead = current;

        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();

        if (fifoCount < packetSize) {
            return false;
        }

        // Check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // Reset so we can continue cleanly
            mpu.resetFIFO();
            // Serial.println("FIFO overflow! " + String(fifoCount));
        } else if (mpuIntStatus & 0x02) {
            mpu.getFIFOBytes(fifoBuffer, packetSize);

            // Track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            y = int((ypr[0] * 180 / M_PI) + 360) % 360;
            p = int((ypr[1] * 180 / M_PI) + 360) % 360;
            r = int((ypr[2] * 180 / M_PI) + 360) % 360;

            return true;
        }

        return false;
    }

private:

    String* logs;

    MPU6050 mpu;

    // MPU control/status vars
    bool dmpReady = false;  // Set true if DMP init was successful
    uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
    uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // Count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

    double lastRead = millis();
};
