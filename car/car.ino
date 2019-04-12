#include <pcf8574_esp.h>
#include <Servo.h>

#include "communicator.h"
#include "navigator.h"
#include "ota.h"   
#include "sensors/mpu.h"
#include "sensors/optical.h"
#include "sensors/encoder.h"
#include "sensors/black.h"
#include "sensors/ultrasonic.h"
#include "utils/constants.h"

//
// Variables & Objects
//
Communicator com;
Navigator nav;
MPUSensor mpu;
BlackSensor blk;
UltrasonicSensor uls;
OTAHandler ota;
Encoder len;
Encoder ren;
Servo servo;

PCF857x pcf1(PCF1_ADDRESS, &Wire);

bool setupState = false;

//
// ISRs
//
void leftEncoderISR() {
    len.ticksHandler();
} 

void rightEncoderISR() {
    ren.ticksHandler();
} 

//
// Logic
//
void setup() {
    // Initialize Serial connection
    // Serial.begin(BAUD_RATE);

    // Initialize I2C Bus
    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialize connection
    setupState = com.setup();
    
    // Dummy check
    for (int i = 0; i < 10000; ++i) {
        ota.handle();
    }

    // Initialize PCFs
    pcf1.begin(0x10); // P0-P3 output, P4 input

    // Initialize OTA
    ota.setup();

    // Initialize sensors
    mpu.setup();
    uls.setup();
    blk.setup(&pcf1);

    // Initialize encoders
    len.setup(LEFT_ENC);
    ren.setup(RGHT_ENC);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RGHT_ENC), rightEncoderISR, CHANGE);

    // Initialize motors
    nav.setup(&com, &pcf1, &len, &ren);

    // Initialize servo
    servo.attach(SERVO_PIN);
    servo.write(SERVO_DOWN_ANGLE);
}

void loop() {
    ota.handle();

    if(!setupState)
        return;

    //
    // Read sensors
    //
    
    // MPU
    double y, p, r;
    if(!mpu.read(y, p, r)) {
        return;
    }

    // Black sensor
    bool isBlack;
    blk.read(isBlack);

    // Ultrasonic
    double distance;
    uls.read(distance);

    //
    // Server commands
    //
    int msg = com.receive();

    switch (msg) {
        case STOP:
            nav.stop(y);
            com.send(String("Stop!"));
        break;

        case FORWARD:
            nav.forward(y);
            com.send(String("Forward!"));
        break;

        case BACKWARD:
            nav.backward(y);
            com.send(String("Backward!"));
        break;

        case LEFT:
            nav.left(y);
            com.send(String("Left!"));
        break;

        case RIGHT:
            nav.right(y);
            com.send(String("Right!"));
        break;

        case UP:
            servo.write(SERVO_UP_ANGLE);
            com.send(String("Up!"));
        break;

        case DOWN:
            servo.write(SERVO_DOWN_ANGLE);
            com.send(String("Down!"));
        break;
    }

    // Navigation
    nav.navigate(y);
}
