#pragma once

#include "constants.h"

class Navigator {
public:

    void setup() {      
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
    
    void stop() {        
        Serial.println("Stop");
        digitalWrite(LEFT_DIR1, LOW);
        digitalWrite(LEFT_DIR2, LOW);
        digitalWrite(RGHT_DIR1, LOW);
        digitalWrite(RGHT_DIR2, LOW);
    }
    
    void forward() {        
        Serial.println("Forward");
        digitalWrite(LEFT_DIR1, HIGH);
        digitalWrite(LEFT_DIR2, LOW);
        digitalWrite(RGHT_DIR1, HIGH);
        digitalWrite(RGHT_DIR2, LOW);
    }
    
    void backward() {
        Serial.println("Backward");
        digitalWrite(LEFT_DIR1, LOW);
        digitalWrite(LEFT_DIR2, HIGH);
        digitalWrite(RGHT_DIR1, LOW);
        digitalWrite(RGHT_DIR2, HIGH);          
    }

    void left() {
        Serial.println("Left");
        analogWrite(LEFT_SPED, 255);
        analogWrite(RGHT_SPED, 100);        
    }

    void right() {
        Serial.println("Right");
        analogWrite(LEFT_SPED, 100);
        analogWrite(RGHT_SPED, 255);         
    }
};
