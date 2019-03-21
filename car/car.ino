#include <pcf8574_esp.h>

#include "communicator.h"
#include "navigator.h"
#include "ota.h"
#include "sensors/mpu.h"
#include "sensors/optical.h"
#include "sensors/encoder.h"
#include "utils/constants.h"

//
// Variables & Objects
//
Communicator com;
Navigator nav;
MPUSensor mpu;
OTAHandler ota;
Encoder len;
Encoder ren;

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
    Serial.begin(BAUD_RATE);

    // Initialize I2C Bus
    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialize connection
    setupState = com.setup();
    
    // Dummy check
    for (int i = 0; i < 10000; ++i) {
        ota.handle();
    }

    // Initialize PCFs
    pcf1.begin(0); // 0 means all 8 pins are outputs
    
    // Initialize OTA
    ota.setup();

    // Initialize sensors
    mpu.setup();

    // Initialize encoders
    len.setup(LEFT_ENC);
    ren.setup(RGHT_ENC);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RGHT_ENC), rightEncoderISR, CHANGE);

    // Initialize motors
    nav.setup(&com, &pcf1, &len, &ren);
}

int i = 0;

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
    }

    // Navigation
    nav.navigate(y);
}