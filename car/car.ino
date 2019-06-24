#include <pcf8574_esp.h>
#include <Servo.h>

#include "communicator.h"
#include "navigator.h"
#include "ota.h"
#include "sensors/encoder.h"
#include "sensors/black.h"
#include "sensors/battery.h"
#include "sensors/ultrasonic.h"
#include "sensors/mpu.h"
#include "utils/constants.h"

extern "C" {
  #include "user_interface.h"
}

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
MPUSensor mpu;

PCF857x pcf1(PCF1_ADDRESS, &Wire);

bool setupState = false;
bool moving = false;
bool blocked = false;

uint8_t batteryLevel;

LIGHT_MODE redLed = LIGHT_MODE::OFF;
LIGHT_MODE blueLed = LIGHT_MODE::OFF;

int lastRedLedValue = LOW;
int lastBlueLedValue = LOW;
double lastRedLedChange = millis();
double lastBlueLedChange = millis();

double y, p, r;
double distance;

double lastSend = millis(); // ToDo: remove

String logs = "";

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
void receive(SERVER_TASKS task);
void updateLights();

void setup() {
    // Initialize I2C Bus
    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialize connection
    setupState = com.setup(&receive);

    // MPU setup
    mpu.setup();

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
    // com.send(MSG_SET);

    // Setup PWM freq
    analogWriteFreq(PWM_FREQUENCY);

    // LEDs
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, FUNCTION_3);
    pinMode(RED_LED_PIN, OUTPUT);

    for(int i = 0; i < 3000; ++i) {
        ota.handle();
    }

    wifi_set_sleep_type(NONE_SLEEP_T);
}

void loop() {
    ota.handle();

    if(!setupState)
        return;

    //
    // Read sensors
    //

    // MPU
    mpu.read(y, p, r);

    // Black sensors
    bool isBackLeftBlack, isBackRightBlack, isFrontLeftBlack, isFrontRightBlack, isFrontCenterBlack;
    blblk.read(isBackLeftBlack);
    brblk.read(isBackRightBlack);
    flblk.read(isFrontLeftBlack);
    frblk.read(isFrontRightBlack);
    fcblk.read(isFrontCenterBlack);

    // Ultrasonic
    uls.read(distance);
    if (!blocked && distance <= MIN_DISTANCE) {
        blocked = true;

        com.sendBlockingState(BLOCKING_MODE::BLOCKED);
    } else if (blocked && distance > MIN_DISTANCE) {
        blocked = false;

        com.sendBlockingState(BLOCKING_MODE::UNBLOCKED);
    }

    // Battery
    uint8_t level;
    bat.read(level);

    if (level != batteryLevel) {
        batteryLevel = level;

        com.sendBatteryLevel(batteryLevel);
    }

    // Light LEDs
    updateLights();

    // if (millis() - lastSend > 1000) {
    //     unsigned long lTicks, rTicks;

    //     noInterrupts();
    //     len.read(lTicks);
    //     ren.read(rTicks);
    //     interrupts();

    //     com.sendStr(
    //         "DEBUG:\nAngle: " + String(y) +
    //         "\nSensors: " + String(isFrontLeftBlack) +
    //         " " + String(isFrontCenterBlack) +
    //         " " + String(isFrontRightBlack) +
    //         " " + String(isBackLeftBlack) +
    //         " " + String(isBackRightBlack) +
    //         " Lticks: " + lTicks +
    //         " Rticks: " + rTicks +
    //         " Sonic: " + distance +
    //         "\n\n");

    //     lastSend = millis();
    // }

    //
    // Logic
    //

    // Server commands
    com.loop();

    // Navigation
    nav.navigate(distance, y, isFrontCenterBlack, isFrontLeftBlack, isFrontRightBlack, isBackLeftBlack, isBackRightBlack, logs);

    if (moving && nav.getState() == IDLE) {
        moving = false;

        com.sendDone();
    }

    if (logs.length() > 0) {
        com.sendStr(logs);

        logs = "";
    }
}

void receive(SERVER_TASKS task) {
    switch (task) {
        case SERVER_TASKS::CONFIG:
            com.sendStr(String("Config!"));
        break;

        case SERVER_TASKS::STOP:
            nav.stop();
            com.sendStr(String("Stop!"));
        break;

        case SERVER_TASKS::MOVE:
            moving = true;
            nav.move();
            com.sendStr(String("Forward!"));
        break;

        case SERVER_TASKS::ROTATE_RIGHT:
            moving = true;
            nav.rotateRight(y);
            com.sendStr(String("Right!"));
        break;

        case SERVER_TASKS::ROTATE_LEFT:
            moving = true;
            nav.rotateLeft(y);
            com.sendStr(String("Left!"));
        break;

        case SERVER_TASKS::RETREAT:
            moving = true;
            nav.retreat(y);
            com.sendStr(String("Backward!"));
        break;

        case SERVER_TASKS::LOAD:
            com.sendStr(String("Load!"));
        break;

        case SERVER_TASKS::OFFLOAD:
            com.sendStr(String("Offload!"));
        break;

        case SERVER_TASKS::RED_LED_OFF:
            redLed = LIGHT_MODE::OFF;
            com.sendStr(String("Red LED off!"));
        break;

        case SERVER_TASKS::RED_LED_ON:
            redLed = LIGHT_MODE::ON;
            com.sendStr(String("Red LED on!"));
        break;

        case SERVER_TASKS::RED_LED_FLASH:
            redLed = LIGHT_MODE::FLASH;
            com.sendStr(String("Red LED flash!"));
        break;

        case SERVER_TASKS::BLUE_LED_OFF:
            blueLed = LIGHT_MODE::OFF;
            com.sendStr(String("Blue LED off!"));
        break;

        case SERVER_TASKS::BLUE_LED_ON:
            blueLed = LIGHT_MODE::ON;
            com.sendStr(String("Blue LED on!"));
        break;

        case SERVER_TASKS::BLUE_LED_FLASH:
            blueLed = LIGHT_MODE::FLASH;
            com.sendStr(String("Blue LED flash!"));
        break;
    }
}

void updateLights() {
    if (redLed == LIGHT_MODE::OFF && lastRedLedValue != LOW) {
        lastRedLedValue = LOW;

        digitalWrite(RED_LED_PIN, lastRedLedValue);
    } else if (redLed == LIGHT_MODE::ON && lastRedLedValue != HIGH) {
        lastRedLedValue = HIGH;

        digitalWrite(RED_LED_PIN, lastRedLedValue);
    } else if (redLed == LIGHT_MODE::FLASH && millis() - lastRedLedChange > FLASH_PERIOD) {
        lastRedLedValue = !lastRedLedValue;

        digitalWrite(RED_LED_PIN, lastRedLedValue);
    }

    if (blueLed == LIGHT_MODE::OFF && lastBlueLedValue != LOW) {
        lastBlueLedValue = LOW;

        digitalWrite(BLUE_LED_PIN, lastBlueLedValue);
    } else if (blueLed == LIGHT_MODE::ON && lastBlueLedValue != HIGH) {
        lastBlueLedValue = HIGH;

        digitalWrite(BLUE_LED_PIN, lastBlueLedValue);
    } else if (blueLed == LIGHT_MODE::FLASH && millis() - lastBlueLedChange > FLASH_PERIOD) {
        lastBlueLedValue = !lastBlueLedValue;

        digitalWrite(BLUE_LED_PIN, lastBlueLedValue);
    }
}