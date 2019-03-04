#include "constants.h"
#include "communicator.h"
#include "navigator.h"

Communicator comm;
Navigator nav;

boolean setupState = false;

void setup() {
    // Initialise Serial connection
    Serial.begin(BAUD_RATE);

    // Initialize connection
    setupState = comm.setup();

    //
    // Motors
    //
    
    // Motor pins
    pinMode(LEFT_DIR1, OUTPUT);
    pinMode(LEFT_DIR2, OUTPUT);
    pinMode(LEFT_SPED, OUTPUT);
    pinMode(RGHT_DIR1, OUTPUT);
    pinMode(RGHT_DIR2, OUTPUT);
    pinMode(RGHT_SPED, OUTPUT);

    // Enable the two motors
    digitalWrite(LEFT_SPED, HIGH);
    digitalWrite(RGHT_SPED, HIGH);
    digitalWrite(LEFT_DIR1, LOW);
    digitalWrite(LEFT_DIR2, LOW);
    digitalWrite(RGHT_DIR1, LOW);
    digitalWrite(RGHT_DIR2, LOW);
}

void loop() {
    if(setupState) {

        //
        // Server commands
        //
        int msg = comm.receive();

        switch (msg) {
            case STOP:
                Serial.println("Stop");
                digitalWrite(LEFT_DIR1, LOW);
                digitalWrite(LEFT_DIR2, LOW);
                digitalWrite(RGHT_DIR1, LOW);
                digitalWrite(RGHT_DIR2, LOW);
            break;

            case FORWARD:
                Serial.println("Forward");
                digitalWrite(LEFT_DIR1, HIGH);
                digitalWrite(LEFT_DIR2, LOW);
                digitalWrite(RGHT_DIR1, HIGH);
                digitalWrite(RGHT_DIR2, LOW);
            break;

            case BACKWARD:
                Serial.println("Backward");
                digitalWrite(LEFT_DIR1, LOW);
                digitalWrite(LEFT_DIR2, HIGH);
                digitalWrite(RGHT_DIR1, LOW);
                digitalWrite(RGHT_DIR2, HIGH);  
            break;

            case LEFT:
                Serial.println("Left");
                analogWrite(LEFT_SPED, 255);
                analogWrite(RGHT_SPED, 100);       
            break;

            case RIGHT:
                Serial.println("Right");
                analogWrite(LEFT_SPED, 100);
                analogWrite(RGHT_SPED, 255);       
            break;
        }

        //
        // Local logic
        //
        
    }
     
    delay(10);
}
