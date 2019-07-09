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
bool failure = false;

double ongoingTimer = 0;

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
bool obtainReferenceAngle();
void readSensors();
void ICACHE_RAM_ATTR leftEncoderISR();
void ICACHE_RAM_ATTR rightEncoderISR();
void updateLights();
void debug();

void setup() {
    // Initialize I2C Bus
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_CLOCK);

    // Initialize connection
    setupState = com.setup(&receive, &serverConnected, &serverDisconnected);
    com.loop();

    // Initialize PCFs
    pcf1.begin(PCF1_CONFIGS); // P0-P3 output, P4-P7 input

    // Initialize OTA
    ota.setup();

    // Initialize sensors
    mpu.setup(failure);
    uls.setup();
    bat.setup(BATTERY_SENSOR_PIN);

    pinMode(FRT_CNT_BLACK_SENSOR_PIN, INPUT_PULLUP);

    // Initialize encoders
    len.setup(LEFT_ENC);
    ren.setup(RGHT_ENC);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RGHT_ENC), rightEncoderISR, CHANGE);

    // Initialize motors
    nav.setup(&pcf1, &len, &ren, &blocked, &logs);

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

    failure |= !setupState;

    // Set navigator reference angle
    failure |= !obtainReferenceAngle();
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
    debug();

    // Navigation
    bool preExecutionBlocked = blocked;
    ExecutionState executionState;
    nav.navigate(obstacleDistance, yaw, blackSensors, executionState);

    if (executionState.state == EXECUTION_STATE::FINISHED) {
        com.sendDone();
    } else if (executionState.state == EXECUTION_STATE::PAUSE) {
        if (blocked != preExecutionBlocked) {
            com.sendBlockingState(blocked ? BLOCKING_MODE::BLOCKED : BLOCKING_MODE::UNBLOCKED);
        }
    } else if (executionState.state == EXECUTION_STATE::ERROR) {
        com.sendError(executionState.error);

        failure = true;
    } else if (executionState.state == EXECUTION_STATE::ONGOING) {
        if (millis() - ongoingTimer > ONGOING_TIMEOUT) {
            nav.terminate();
        }
    }

    if (executionState.state != EXECUTION_STATE::ONGOING) {
        ongoingTimer = millis();

        debug();
    }

    yield();
}

void receive(SERVER_TASKS task) {
    switch (task) {
        case SERVER_TASKS::CONFIG:
            logs.concat("Config!\n");

            failure = false;
            nav.config();
        break;

        case SERVER_TASKS::STOP:
            logs.concat("Stop!\n");

            nav.stop();
        break;

        case SERVER_TASKS::MOVE:
            logs.concat("Forward!\n");

            nav.move(yaw);
        break;

        case SERVER_TASKS::ROTATE_RIGHT:
            logs.concat("Right!\n");

            nav.rotateRight(yaw);
        break;

        case SERVER_TASKS::ROTATE_LEFT:
            logs.concat("Left!\n");

            nav.rotateLeft(yaw);
        break;

        case SERVER_TASKS::RETREAT:
            logs.concat("Retreat!\n");

            nav.retreat(yaw);
        break;

        case SERVER_TASKS::RECOVER_MOVE:
            logs.concat("Recover forward!\n");

            nav.move(yaw, true);
        break;

        case SERVER_TASKS::RECOVER_ROTATE_RIGHT:
            logs.concat("Recover right!\n");

            nav.rotateRight(yaw, true);
        break;

        case SERVER_TASKS::RECOVER_ROTATE_LEFT:
            logs.concat("Recover left!\n");

            nav.rotateLeft(yaw, true);
        break;

        case SERVER_TASKS::RECOVER_RETREAT:
            logs.concat("Recover retreat!\n");

            nav.retreat(yaw, true);
        break;

        case SERVER_TASKS::LOAD:
            logs.concat("Load!\n");

            com.sendDone();
        break;

        case SERVER_TASKS::OFFLOAD:
            logs.concat("Offload!\n");

            com.sendDone();
        break;

        case SERVER_TASKS::RED_LED_OFF:
            logs.concat("Red LED off!\n");

            redLed = LIGHT_MODE::OFF;
        break;

        case SERVER_TASKS::RED_LED_ON:
            logs.concat("Red LED on!\n");

            redLed = LIGHT_MODE::ON;
        break;

        case SERVER_TASKS::RED_LED_FLASH:
            logs.concat("Red LED flash!\n");

            redLed = LIGHT_MODE::FLASH;
        break;

        case SERVER_TASKS::BLUE_LED_OFF:
            logs.concat("Blue LED off!\n");

            blueLed = LIGHT_MODE::OFF;
        break;

        case SERVER_TASKS::BLUE_LED_ON:
            logs.concat("Blue LED on!\n");

            blueLed = LIGHT_MODE::ON;
        break;

        case SERVER_TASKS::BLUE_LED_FLASH:
            logs.concat("Blue LED flash!\n");

            blueLed = LIGHT_MODE::FLASH;
        break;
    }
}

void serverConnected() {
    logs.concat("serverConnected()\n");
}

void serverDisconnected() {
    logs.concat("serverDisconnected()\n");
    nav.stop();
}

bool obtainReferenceAngle() {
    int j = 0;

    while(j++ < MPI_INIT_READINGS) {
        int i = 0;

        while(!mpu.read(yaw, pitch, roll) && i++ < 10) {
            delay(200);
        }

        if (i == 10)
            return false;

        delay(1000);
    }

    nav.setReferenceAngle(yaw);

    return true;
}

void readSensors() {
    // MPU
    mpu.read(yaw, pitch, roll);

    // Black sensors
    blackSensors[1] = digitalRead(FRT_CNT_BLACK_SENSOR_PIN) == LOW;

    uint8_t pcfReading = pcf1.read8();
    blackSensors[0] = (pcfReading & (1 << FRT_LFT_BLACK_SENSOR_PIN)) == 0;
    blackSensors[2] = (pcfReading & (1 << FRT_RGT_BLACK_SENSOR_PIN)) == 0;
    blackSensors[3] = (pcfReading & (1 << BAK_LFT_BLACK_SENSOR_PIN)) == 0;
    blackSensors[4] = (pcfReading & (1 << BAK_RGT_BLACK_SENSOR_PIN)) == 0;

    blackSensors[0] = (FRT_LFT_BLACK_SENSOR_INV ? !blackSensors[0] : blackSensors[0]);
    blackSensors[1] = (FRT_CNT_BLACK_SENSOR_INV ? !blackSensors[1] : blackSensors[1]);
    blackSensors[2] = (FRT_RGT_BLACK_SENSOR_INV ? !blackSensors[2] : blackSensors[2]);
    blackSensors[3] = (BAK_LFT_BLACK_SENSOR_INV ? !blackSensors[3] : blackSensors[3]);
    blackSensors[4] = (BAK_LFT_BLACK_SENSOR_INV ? !blackSensors[4] : blackSensors[4]);

    // Ultrasonic
    uls.read(obstacleDistance);

    // if (!blocked && obstacleDistance <= MIN_DISTANCE) { // On block state change to BLOCKED
    //     blocked = true;

    //     com.sendBlockingState(BLOCKING_MODE::BLOCKED);
    // } else
    if (blocked && obstacleDistance > MIN_DISTANCE) { // On block state change to UNBLOCKED
        blocked = false;

        com.sendBlockingState(BLOCKING_MODE::UNBLOCKED);
    }

    // Battery
    // uint8_t level;
    // bat.read(level);

    // if (level != batteryLevel) { // On battery level change
    //     batteryLevel = level;

    //     com.sendBatteryLevel(batteryLevel);
    //     logs.concat("New battery level: " + String(batteryLevel) + "\n");
    // }
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

    // if (batteryLevel < BATTERY_WARNING_LEVEL) {
    //     redLed = LIGHT_MODE::FLASH;
    // }

    if (blocked || failure || !com.isConnected()) {
        redLed = LIGHT_MODE::ON;
    }

    if (com.isConnected() && !blocked && !failure/* && batteryLevel > BATTERY_WARNING_LEVEL*/) {
        redLed = LIGHT_MODE::OFF;
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

        logs.concat(
           "DEBUG:\nAngle: " + String(yaw) +
            "\nRefAngle: " + String(nav.getReferenceAngle()) +
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

            com.sendStr("\n\n\nLOGS::\n" + logs + "\n\n\n");

            logs = "";
        }

        lastSend = millis();
    }
}
