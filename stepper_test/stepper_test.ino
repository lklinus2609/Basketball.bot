#include <AccelStepper.h>

// Pins
#define STEP_PIN A4
#define DIR_PIN A5
#define ENABLE_PIN 12

// Create stepper instance
AccelStepper loader(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Stepper Test Starting...");
    Serial.println("Pins: STEP=A4, DIR=A5, EN=12");

    // Enable Driver
    // pinMode(ENABLE_PIN, OUTPUT);
    // digitalWrite(ENABLE_PIN, LOW); // LOW = Enabled

    // Configure Stepper
    loader.setMaxSpeed(10000);
    loader.setAcceleration(5000); // Using the 5000 value we set earlier
}

void loop() {
    if (loader.distanceToGo() == 0) {
        delay(5000); // Wait 5s
        Serial.println("Moving 800 steps...");
        loader.move(800); // Move 90 degrees (assuming 1/16 microstepping)
    }
    loader.run();
}
