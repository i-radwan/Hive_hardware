#include "utils/constants.h"
#include "communicator.h"
#include "navigator.h"
#include "ota.h"
#include "sensors/mpu.h"

Communicator comm;
Navigator nav;
MPUSensor mpu;
OTAHandler ota;

bool setupState = false;

void setup() {
    // Initialize Serial connection
    Serial.begin(BAUD_RATE);

    // Initialize connection
    setupState = comm.setup();

    // Initialize OTA
    ota.setup();

    // Initialize sensors
    mpu.setup();

    // Initialize motors
    nav.setup();
}

void loop() {
    ota.handle();

    if(!setupState)
        return;

    // Read sensors
    double y, p, r;
    if (mpu.read(y, p, r)) {
        // comm.send(((String) y + " - " + (String) nav.getReferenceAngle()).c_str());
    }

    // Server commands
    int msg = comm.receive();

    switch (msg) {
        case STOP:
            nav.stop();
        break;

        case FORWARD:
            nav.forward(y);
        break;

        case BACKWARD:
            nav.backward(y);
        break;

        case LEFT:
            nav.left(y);
        break;

        case RIGHT:
            nav.right(y);
        break;
    }

    // Navigation
    nav.move(y);

    delay(10);
}
