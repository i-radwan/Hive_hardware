extern "C" {
  #include "user_interface.h"
}

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

// #define HIVE_DEBUG

// ====================
// Variables & Objects

// Objects
Communicator com;
Navigator nav;
BlackSensor blblk;
BlackSensor brblk;
BlackSensor flblk;
BlackSensor frblk;
BlackSensor fcblk;
UltrasonicSensor uls;
BatterySensor bat;
Encoder len;
Encoder ren;
MPUSensor mpu;
OTAHandler ota;

PCF857x pcf1(PCF1_ADDRESS, &Wire);

// Battery
uint8_t batteryLevel;

// LEDs
LIGHT_MODE redLed = LIGHT_MODE::OFF;
LIGHT_MODE blueLed = LIGHT_MODE::OFF;

int lastRedLedValue = LOW;
int lastBlueLedValue = LOW;
double lastRedLedChange = millis();
double lastBlueLedChange = millis();

// State variables
bool setupState = false;
bool blocked = false;

// Sensor readings
double yaw, pitch, roll;
double obstacleDistance;
bool blackSensors[5];

// Logs
double lastSend = millis(); // ToDo: remove
String logs = "";


// ====================
// Functions

void receive(SERVER_TASKS task);
void serverConnected();
void serverDisconnected();
void readSensors();
void ICACHE_RAM_ATTR leftEncoderISR();
void ICACHE_RAM_ATTR rightEncoderISR();
void updateLights();
void debug();

void setup() {
    // Initialize I2C Bus
    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialize connection
    setupState = com.setup(&receive, &serverConnected, &serverDisconnected);

    // Initialize PCFs
    pcf1.begin(PCF1_CONFIGS); // P0-P3 output, P4-P7 input

    // Initialize OTA
    ota.setup();

    // Initialize sensors
    mpu.setup();
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
    nav.setup(&pcf1, &len, &ren, &logs);

    // Setup PWM freq
    analogWriteFreq(PWM_FREQUENCY);

    // LEDs
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, FUNCTION_3);
    pinMode(RED_LED_PIN, OUTPUT);

    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);

    // Prevent sleeping to enhance communication speed
    wifi_set_sleep_type(NONE_SLEEP_T);
}

void loop() {
    ota.handle();

    if(!setupState)
        return;

    // Server commands
    com.loop();

    // Read sensors
    readSensors();

    // Light LEDs
    updateLights();

    // Printing debugging info
    // debug();

    // Navigation
    ExecutionState executionState;
    nav.navigate(obstacleDistance, yaw, blackSensors, executionState);

    if (executionState.state == EXECUTION_STATE::FINISHED) {
        com.issueDone();
    } else if (executionState.state == EXECUTION_STATE::ERROR) {
        com.sendError();
    }

    if (executionState.state != EXECUTION_STATE::ONGOING) {
        debug();
    }

    yield();
}

void receive(SERVER_TASKS task) {
    switch (task) {
        case SERVER_TASKS::CONFIG:
            nav.config();
            com.sendStr(String("Config!"));
        break;

        case SERVER_TASKS::STOP:
            nav.stop();
            com.sendStr(String("Stop!"));
        break;

        case SERVER_TASKS::MOVE:
            nav.move(yaw);
            com.sendStr(String("Forward!"));
        break;

        case SERVER_TASKS::ROTATE_RIGHT:
            nav.rotateRight(yaw);
            com.sendStr(String("Right!"));
        break;

        case SERVER_TASKS::ROTATE_LEFT:
            nav.rotateLeft(yaw);
            com.sendStr(String("Left!"));
        break;

        case SERVER_TASKS::RETREAT:
            nav.retreat(yaw);
            com.sendStr(String("Backward!"));
        break;

        case SERVER_TASKS::LOAD:
            com.sendDone();
            com.sendStr(String("Load!"));
        break;

        case SERVER_TASKS::OFFLOAD:
            com.sendDone();
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

void serverConnected() {
    redLed = LIGHT_MODE::OFF;
}

void serverDisconnected() {
    redLed = LIGHT_MODE::ON;
    nav.stop();
}

void readSensors() {
    // MPU
    mpu.read(yaw, pitch, roll);

    // Black sensors
    flblk.read(blackSensors[0]);
    fcblk.read(blackSensors[1]);
    frblk.read(blackSensors[2]);
    blblk.read(blackSensors[3]);
    brblk.read(blackSensors[4]);

    // Ultrasonic
    uls.read(obstacleDistance);

    if (!blocked && obstacleDistance <= MIN_DISTANCE) { // On block state change to BLOCKED
        blocked = true;

        com.sendBlockingState(BLOCKING_MODE::BLOCKED);
    } else if (blocked && obstacleDistance > MIN_DISTANCE) { // On block state change to UNBLOCKED
        blocked = false;

        com.sendBlockingState(BLOCKING_MODE::UNBLOCKED);
    }

    // Battery
    uint8_t level;
    bat.read(level);

    if (level != batteryLevel) { // On battery level change
        batteryLevel = level;

        com.sendBatteryLevel(batteryLevel);
        com.sendStr("New battery level: " + String(batteryLevel) + "\n");
    }
}

void ICACHE_RAM_ATTR leftEncoderISR() {
    len.ticksHandler();
}

void ICACHE_RAM_ATTR rightEncoderISR() {
    ren.ticksHandler();
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
        lastRedLedChange = millis();

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
        lastBlueLedChange = millis();

        digitalWrite(BLUE_LED_PIN, lastBlueLedValue);
    }
}

void debug() {
    if (millis() - lastSend > 1000) {

        #ifdef HIVE_DEBUG
        unsigned long lTicks, rTicks;

        noInterrupts();
        len.read(lTicks);
        ren.read(rTicks);
        interrupts();

        com.sendStr(
            "DEBUG:\nAngle: " + String(yaw) +
            "\nSensors: " + String(blackSensors[0]) +
            " " + String(blackSensors[1]) +
            " " + String(blackSensors[2]) +
            " " + String(blackSensors[3]) +
            " " + String(blackSensors[4]) +
            " Lticks: " + lTicks +
            " Rticks: " + rTicks +
            " Sonic: " + obstacleDistance +
            "\n\n");
        #endif

        // ToDo: remove
        if (logs.length() > 0) {
            // com.sendStr("\n\n\nLOGS::\n" + logs + "\n\n\n");

            logs = "";
        }

        lastSend = millis();
    }
}