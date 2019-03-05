#include "utils/constants.h"
#include "communicator.h"
#include "navigator.h"
#include "sensors/mpu.h"

Communicator comm;
Navigator nav;
MPUSensor mpu;

boolean setupState = false;

void setup() {
    // Initialize Serial connection
    Serial.begin(BAUD_RATE);

    // Initialize connection
    setupState = comm.setup();

    // Initialize motors
    nav.setup();

    // Initialize sensors
    mpu.setup();
}

void loop() {
    if(!setupState)
        return;

    //
    // Server commands
    //
    int msg = comm.receive();

    switch (msg) {
        case STOP:
            nav.stop();
        break;

        case FORWARD:
            nav.forward();
        break;

        case BACKWARD:
            nav.backward();
        break;

        case LEFT:
            nav.left();
        break;

        case RIGHT:
            nav.right();
        break;
    }

    //
    // Local logic
    //

    // Read sensors
    double ax, ay, az, gx, gy, gz;
    mpu.read(ax, ay, az, gx, gy, gz);

    // Serial.println("Reading Sensors...");
    // Serial.print("Ax: " + (String) ax);
    // Serial.print("Ay: " + (String) ay);
    // Serial.print("Az: " + (String) az);
    // Serial.print("Gx: " + (String) gx);
    // Serial.print("Gy: " + (String) gy);
    // Serial.print("Gz: " + (String) gz);

    // Navigation
    nav.move(0);

    delay(10);
}
