#pragma once

#include <Wire.h>

#include "../utils/constants.h"

class MPUSensor {
public:
    
    void setup() {
        Wire.begin(MPU_I2C_SDA, MPU_I2C_SCL);
        
        delay(150);
        
        I2C_Write(MPU_ADDRESS, MPU_REG_SMPLRT_DIV, 0x07);
        I2C_Write(MPU_ADDRESS, MPU_REG_PWR_MGMT_1, 0x01);
        I2C_Write(MPU_ADDRESS, MPU_REG_PWR_MGMT_2, 0x00);
        I2C_Write(MPU_ADDRESS, MPU_REG_CONFIG, 0x00);
        I2C_Write(MPU_ADDRESS, MPU_REG_GYRO_CONFIG, 0x00); // Set +/-250 degree/second full scale
        I2C_Write(MPU_ADDRESS, MPU_REG_ACCEL_CONFIG, 0x00); // Set +/- 2g full scale
        I2C_Write(MPU_ADDRESS, MPU_REG_FIFO_EN, 0x00);
        I2C_Write(MPU_ADDRESS, MPU_REG_INT_ENABLE, 0x01);
        I2C_Write(MPU_ADDRESS, MPU_REG_SIGNAL_PATH_RESET, 0x00);
        I2C_Write(MPU_ADDRESS, MPU_REG_USER_CTRL, 0x00);
    }
    
    void read(double& ax, double& ay, double& az, double& gx, double& gy, double& gz) {
        Wire.beginTransmission(MPU_ADDRESS);
        Wire.write(MPU_REG_ACCEL_XOUT_H);
        Wire.endTransmission();
        
        Wire.requestFrom(MPU_ADDRESS, (int) 14);
        ax = (((int) Wire.read() << 8) | Wire.read()) / MPU_ACCEL_SCALE_FACTOR;
        ay = (((int) Wire.read() << 8) | Wire.read()) / MPU_ACCEL_SCALE_FACTOR;
        az = (((int) Wire.read() << 8) | Wire.read()) / MPU_ACCEL_SCALE_FACTOR;
        gx = (((int) Wire.read() << 8) | Wire.read()) / MPU_GYRO_SCALE_FACTOR;
        gy = (((int) Wire.read() << 8) | Wire.read()) / MPU_GYRO_SCALE_FACTOR;
        gz = (((int) Wire.read() << 8) | Wire.read()) / MPU_GYRO_SCALE_FACTOR;
    }

private:
    void I2C_Write(int deviceAddress, int regAddress, int data){
        Wire.beginTransmission(deviceAddress);
        Wire.write(regAddress);
        Wire.write(data);
        Wire.endTransmission();
    }    
};
