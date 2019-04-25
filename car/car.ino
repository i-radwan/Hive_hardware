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
// MPUSensor mpu;
BlackSensor blblk;
BlackSensor brblk;
BlackSensor flblk;
BlackSensor frblk;
BlackSensor fcblk;
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
    pcf1.begin(0xF0); // P0-P3 output, P4-P7 input

    // Initialize OTA
    ota.setup();

    // Initialize sensors
    // mpu.setup();
    uls.setup();
    blblk.setup(&pcf1, BAK_LFT_BLACK_SENSOR_PIN, BAK_LFT_BLACK_SENSOR_INV, BAK_LFT_BLACK_SENSOR_PCF);
    brblk.setup(&pcf1, BAK_RGT_BLACK_SENSOR_PIN, BAK_RGT_BLACK_SENSOR_INV, BAK_RGT_BLACK_SENSOR_PCF);
    flblk.setup(&pcf1, FRT_LFT_BLACK_SENSOR_PIN, FRT_LFT_BLACK_SENSOR_INV, FRT_LFT_BLACK_SENSOR_PCF);
    frblk.setup(&pcf1, FRT_RGT_BLACK_SENSOR_PIN, FRT_RGT_BLACK_SENSOR_INV, FRT_RGT_BLACK_SENSOR_PCF);
    fcblk.setup(&pcf1, FRT_CNT_BLACK_SENSOR_PIN, FRT_CNT_BLACK_SENSOR_INV, FRT_CNT_BLACK_SENSOR_PCF);

    // Initialize encoders
    len.setup(LEFT_ENC);
    ren.setup(RGHT_ENC);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RGHT_ENC), rightEncoderISR, CHANGE);

    // Initialize motors
    nav.setup(&com, &pcf1, &len, &ren);

    // Initialize servo
    // servo.attach(SERVO_PIN);
    // servo.write(SERVO_DOWN_ANGLE);

    // Send initial ACK
    com.send(MSG_SET);

    // Setup PWM freq
    analogWriteFreq(500);
}

int i = 0;
String logs = "";

void loop() {
    ota.handle();

    if(!setupState)
        return;
    
    //
    // Read sensors
    //
    
    // MPU
    double y = 0, p = 0, r = 0;
    // mpu.read(y, p, r);
    //     return;

    // Black sensor
    bool isBackLeftBlack, isBackRightBlack, isFrontLeftBlack, isFrontRightBlack, isFrontCenterBlack;
    blblk.read(isBackLeftBlack);
    brblk.read(isBackRightBlack);
    flblk.read(isFrontLeftBlack);
    frblk.read(isFrontRightBlack);
    fcblk.read(isFrontCenterBlack);

    // com.send(String(isFrontLeftBlack) + " " + String(isFrontRightBlack) + " " + String(isFrontCenterBlack) + " " + String(isBackLeftBlack) + " " + String(isBackRightBlack));

    // Ultrasonic
    double distance;
    // uls.read(distance);

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
    bool finished = nav.navigate(y, distance, isFrontCenterBlack, isFrontLeftBlack, isFrontRightBlack, isBackLeftBlack, isBackRightBlack, logs);

    delay(50);
    
    if (logs.length() > 0) {
        com.send(String(i++) + " :: " + logs);
        
        logs = "";
    }
    
    if (finished)
        com.send(MSG_ACK);

    // yield();
    delay(10);
}
