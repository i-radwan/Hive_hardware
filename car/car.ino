#include <pcf8574_esp.h>
#include <Servo.h>

#include "communicator.h"
#include "navigator.h"
#include "ota.h"
#include "sensors/encoder.h"
#include "sensors/black.h"
#include "sensors/battery.h"
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
BatterySensor bat;
OTAHandler ota;
Encoder len;
Encoder ren;
Servo servo;

PCF857x pcf1(PCF1_ADDRESS, &Wire);

bool setupState = false;
bool active = false;
bool failure = false;

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
    bat.setup(BATTERY_SENSOR_PIN);
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

    // LEDs
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, FUNCTION_3);
    pinMode(RED_LED_PIN, OUTPUT);

    for(int i = 0; i < 3000; ++i) {
        ota.handle();
    }
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
    // uls.read(distance); // ToDo

    // Battery
    bool isLow;
    // bat.read(isLow); // ToDo

    //
    // Server commands
    //
    MSG msg = com.receive();

    switch (msg) {
        case MSG::STOP:
            nav.stop();
            com.send(String("Stop!"));
        break;

        case MSG::MOVE:
            nav.move();
            com.send(String("Forward!"));
        break;

        case MSG::RETREAT:
            nav.retreat();
            com.send(String("Backward!"));
        break;

        case MSG::ROTATE_LEFT:
            nav.rotateLeft();
            com.send(String("Left!"));
        break;

        case MSG::ROTATE_RIGHT:
            nav.rotateRight();
            com.send(String("Right!"));
        break;
    }

    // Navigation
    bool finished = nav.navigate(distance, isFrontCenterBlack, isFrontLeftBlack, isFrontRightBlack, isBackLeftBlack, isBackRightBlack, logs);

    // Update LEDs, ToDo: read battery level
    digitalWrite(BLUE_LED_PIN, active ? HIGH : LOW);
    digitalWrite(RED_LED_PIN, isLow || failure ? HIGH : LOW);

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