/**
 * Stepper motor control (Lazy Susan and Loader) and Sensor reading
 * Uses AccelStepper library for smooth motion
 * Uses AStar32U4Motors for drive wheels
 */

#include "stepper_and_sensors.h"
#include "config.h"
#include <AccelStepper.h>
#include <AStar32U4Motors.h>

StepperAndSensors steppers_sensors;

// AccelStepper instances
AccelStepper lazy_susan(AccelStepper::DRIVER, LAZY_SUSAN_STEP, LAZY_SUSAN_DIR);
AccelStepper loader(AccelStepper::DRIVER, LOADER_STEP, LOADER_DIR);

// Drive motors using built-in A-Star motor drivers
AStar32U4Motors drive_motors;

StepperAndSensors::StepperAndSensors() {
    loader_ready = true;
    current_lazy_susan_angle = 0.0;
}

void StepperAndSensors::begin() {
    // Setup lazy susan
    pinMode(LAZY_SUSAN_ENABLE, OUTPUT);
    digitalWrite(LAZY_SUSAN_ENABLE, LOW); // Enable (active low for DRV8825)

    lazy_susan.setMaxSpeed(LAZY_SUSAN_MAX_SPEED);
    lazy_susan.setAcceleration(LAZY_SUSAN_ACCELERATION);
    lazy_susan.setCurrentPosition(0);

    // Setup loader
    pinMode(LOADER_ENABLE, OUTPUT);
    digitalWrite(LOADER_ENABLE, LOW); // Enable

    loader.setMaxSpeed(LOADER_MAX_SPEED);
    loader.setAcceleration(LOADER_ACCELERATION);
    loader.setCurrentPosition(0);

    // Initialize drive motors to STOPPED
    drive_motors.setM1Speed(0);
    drive_motors.setM2Speed(0);

    // Setup IR sensors (only 2 sensors)
    pinMode(IR_SENSOR_LEFT, INPUT);
    pinMode(IR_SENSOR_RIGHT, INPUT);

    // Setup ultrasonic sensor
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    digitalWrite(ULTRASONIC_TRIG, LOW);
}

void StepperAndSensors::update() {
    // Update steppers (non-blocking)
    lazy_susan.run();
    loader.run();

    // Update loader ready status
    if (loader.distanceToGo() == 0) {
        loader_ready = true;
    }
}

void StepperAndSensors::rotateLazySusan(float angle_degrees) {
    // Convert degrees to steps
    long target_steps = (long)(angle_degrees * LAZY_SUSAN_STEPS_PER_DEG);

    lazy_susan.moveTo(target_steps);

    // Track current angle
    current_lazy_susan_angle = angle_degrees;
}

void StepperAndSensors::shoot() {
    if (loader_ready) {
        // Move 90 degrees (relative move)
        loader.move(LOADER_STEPS_PER_90_DEG);
        loader_ready = false;

        // After 90°, the loader automatically resets position for continuous rotation
        // Each 90° turn shoots AND loads next ball
    }
}

float StepperAndSensors::getLazySusanAngle() {
    // Return actual angle from step position
    long current_steps = lazy_susan.currentPosition();
    return current_steps / LAZY_SUSAN_STEPS_PER_DEG;
}

bool StepperAndSensors::isLoaderReady() {
    return loader_ready;
}

uint8_t StepperAndSensors::readIR_Left() {
    // TSOP34156 is active-low: LOW = beacon detected
    return digitalRead(IR_SENSOR_LEFT);
}

uint8_t StepperAndSensors::readIR_Right() {
    return digitalRead(IR_SENSOR_RIGHT);
}

uint16_t StepperAndSensors::readUltrasonic() {
    // Trigger ultrasonic pulse
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);

    // Measure echo pulse duration
    unsigned long duration = pulseIn(ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);

    // Calculate distance in cm
    // Distance = (duration * speed_of_sound) / 2
    uint16_t distance_cm = (uint16_t)((duration * SOUND_SPEED_CM_US) / 2.0);

    // Sanity check
    if (distance_cm > 200) {
        distance_cm = 200; // Cap at 2m
    }

    return distance_cm;
}

void StepperAndSensors::setDrive(int8_t left_speed, int8_t right_speed) {
    // Using AStar32U4Motors library
    // Library range: -400 to 400, but we limit to reasonable speed
    // Python sends -127 to 127 range (signed byte)

    // Clamp to max speed of 150 (safe speed based on IRmove test using 75)
    int16_t left_motor = constrain(left_speed, -150, 150);
    int16_t right_motor = constrain(right_speed, -150, 150);

    // M1 = Right motor, M2 = Left motor (typical A-Star configuration)
    drive_motors.setM1Speed(right_motor);
    drive_motors.setM2Speed(left_motor);
}
