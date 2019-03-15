#include <pcf8574_esp.h>

#include "utils/constants.h"
#include "communicator.h"
#include "navigator.h"
#include "ota.h"
#include "sensors/mpu.h"
#include "sensors/optical.h"

Communicator comm;
Navigator nav;
MPUSensor mpu;
OTAHandler ota;

PCF857x pcf1(PCF1_ADDRESS, &Wire);

bool setupState = false;

void setup() {
    // Initialize Serial connection
    Serial.begin(BAUD_RATE);

    // Initialize I2C Bus
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(150);

    // Initialize connection
    setupState = comm.setup();
    
    // Initialize PCFs
    pcf1.begin(0); // 0 means all 8 pins are outputs
    
    // Initialize OTA
    ota.setup();

    // Initialize sensors
    mpu.setup();

    // Initialize motors
    nav.setup(&comm, &pcf1);
}

int i = 0;

void loop() {
    ota.handle();

    if(!setupState)
        return;

    // Read sensors
    double y, p, r;
    if (mpu.read(y, p, r)) {
        // comm.send(((String) y + " - " + (String) nav.getReferenceAngle()));
    }

    // Server commands
    int msg = comm.receive();

    switch (msg) {
        case STOP:
            nav.stop();
            comm.send(String("Stop!"));
        break;

        case FORWARD:
            nav.forward(y);
            comm.send(String("Forward!"));
        break;

        case BACKWARD:
            nav.backward(y);
            comm.send(String("Backward!"));
        break;

        case LEFT:
            nav.left(y);
            comm.send(String("Left!"));
        break;

        case RIGHT:
            nav.right(y);
            comm.send(String("Right!"));
        break;
    }

    // Navigation
    nav.move(y);

    delay(10);
}
