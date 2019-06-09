#include <pcf8574_esp.h>
#include <Servo.h>

#include "communicator.h"
#include "navigator.h"
#include "ota.h"   
#include "sensors/encoder.h"
#include "sensors/black.h"
#include "sensors/ultrasonic.h"
#include "utils/constants.h"

//
// Variables & Objects
//
Communicator com;
Navigator nav;
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
void ICACHE_RAM_ATTR leftEncoderISR() {
    len.ticksHandler();
} 

void ICACHE_RAM_ATTR rightEncoderISR() {
    ren.ticksHandler();
} 

//
// Logic
//
void setup() {
    // Initialize I2C Bus
    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialize connection
    setupState = com.setup();

    // Initialize PCFs
    pcf1.begin(0xF0); // P0-P3 output, P4-P7 input

    // Initialize OTA
    ota.setup();

    // Initialize sensors
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
    nav.setup(&pcf1, &len, &ren);

    // Initialize servo
    // servo.attach(SERVO_PIN);
    // servo.write(SERVO_DOWN_ANGLE);

    // Send initial setup completed signal
    com.send(MSG_SET);

    // Setup PWM freq
    analogWriteFreq(PWM_FREQUENCY);
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
    
    // Black sensors
    bool isBackLeftBlack, isBackRightBlack, isFrontLeftBlack, isFrontRightBlack, isFrontCenterBlack;
    blblk.read(isBackLeftBlack);
    brblk.read(isBackRightBlack);
    flblk.read(isFrontLeftBlack);
    frblk.read(isFrontRightBlack);
    fcblk.read(isFrontCenterBlack);

    // Ultrasonic
    double distance;
    // uls.read(distance);

    //
    // Server commands
    //
    int msg = com.receive();

    switch (msg) {
        case STOP:
            nav.stop();
            com.send(String("Stop!"));
        break;

        case FORWARD:
            nav.forward();
            com.send(String("Forward!"));
        break;

        case BACKWARD:
            nav.backward();
            com.send(String("Backward!"));
        break;

        case LEFT:
            nav.left();
            com.send(String("Left!"));
        break;

        case RIGHT:
            nav.right();
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
    bool finished = nav.navigate(distance, isFrontCenterBlack, isFrontLeftBlack, isFrontRightBlack, isBackLeftBlack, isBackRightBlack, logs);

    // Transmit back to the server
    delay(50);
    
    if (logs.length() > 0) {
        com.send(logs);
        
        logs = "";
    }
    
    if (finished) {
        com.send(MSG_ACK);
    }

    delay(10);
}