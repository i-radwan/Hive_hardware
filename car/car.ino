#include "utils/constants.h"
#include "communicator.h"
#include "navigator.h"
#include "ota.h"
#include "sensors/mpu.h"
#include "sensors/optical.h"

Communicator comm;
Navigator nav;
MPUSensor mpu;
OpticalSensor opt;
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
    opt.setup();

    // Initialize motors
    nav.setup();
}

int i = 0;

void loop() {
    ota.handle();

    if(!setupState)
        return;

    // Read sensors
    double y, p, r;
    if (mpu.read(y, p, r)) {
        // comm.send(((String) y + " - " + (String) nav.getReferenceAngle()).c_str());
    }

    double dx, dy;
    opt.read(dx, dy);

    // Server commands
    int msg = comm.receive();

    switch (msg) {
        case STOP:
            nav.stop();
            comm.send(String("Stop!").c_str());
        break;

        case FORWARD:
            nav.forward(y);
            comm.send(String("Forward!").c_str());
        break;

        case BACKWARD:
            nav.backward(y);
            comm.send(String("Backward!").c_str());
        break;

        case LEFT:
            nav.left(y);
            comm.send(String("Left!").c_str());
        break;

        case RIGHT:
            nav.right(y);
            comm.send(String("Right!").c_str());
        break;
    }

    // Navigation
    nav.move(y, dx, dy);

    delay(10);
}
