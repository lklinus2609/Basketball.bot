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

// Panning constants
const long ENCODER_PAN_RANGE = 363;  // Rotation value for 45 deg sweep
const long SAFETY_LIMIT = 800;       // Hard limit

StepperAndSensors::StepperAndSensors() {
    loader_ready = true;
    current_lazy_susan_angle = 0.0;
    is_panning = false;
    pan_speed = 0;
    pan_direction = 0;
    pan_rotation_target = 0;
    encoder_left = nullptr;
    encoder_right = nullptr;
}

void StepperAndSensors::begin(Encoder* left, Encoder* right) {
    encoder_left = left;
    encoder_right = right;
    
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

    // Initialize drive motors to STOPPED (critical - set multiple times to ensure)
    drive_motors.setM1Speed(0);
    drive_motors.setM2Speed(0);
    delay(10);  // Small delay to ensure motor driver receives command
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
    
    // Update panning logic if active
    if (is_panning) {
        updatePanning();
    }
}

void StepperAndSensors::startPanning(int16_t speed) {
    if (!encoder_left || !encoder_right) return;
    
    is_panning = true;
    pan_speed = abs(speed);
    
    // Reset encoders to 0 for new panning session
    encoder_left->write(0);
    encoder_right->write(0);
    
    // Start panning LEFT initially (standard convention)
    pan_direction = -1;
    pan_rotation_target = -ENCODER_PAN_RANGE;
}

void StepperAndSensors::stopPanning() {
    is_panning = false;
    drive_motors.setM1Speed(0);
    drive_motors.setM2Speed(0);
}

void StepperAndSensors::updatePanning() {
    if (!encoder_left || !encoder_right) return;
    
    // Calculate rotation
    long enc_right = encoder_right->read();
    long enc_left = encoder_left->read();
    long current_rotation = (enc_right - enc_left) / 2;
    
    // Safety bounds check
    if(current_rotation > SAFETY_LIMIT && pan_direction == 1) {
        pan_direction = -1; // Force Left
        pan_rotation_target = -ENCODER_PAN_RANGE;
    }
    else if(current_rotation < -SAFETY_LIMIT && pan_direction == -1) {
        pan_direction = 1; // Force Right
        pan_rotation_target = ENCODER_PAN_RANGE;
    }
    
    // Check limits and reverse
    if(pan_direction < 0 && current_rotation <= pan_rotation_target) {
        // Hit left limit, reverse to right
        pan_direction = 1;
        pan_rotation_target = ENCODER_PAN_RANGE;
    } 
    else if(pan_direction > 0 && current_rotation >= pan_rotation_target) {
        // Hit right limit, reverse to left
        pan_direction = -1;
        pan_rotation_target = -ENCODER_PAN_RANGE;
    }
    
    // Apply motor speed
    // M1 = Right, M2 = Left
    // Rotate Right (CW): M1 backward (-), M2 forward (+)
    // Rotate Left (CCW): M1 forward (+), M2 backward (-)
    
    if(pan_direction > 0) {
        drive_motors.setM1Speed(-pan_speed);
        drive_motors.setM2Speed(pan_speed);
    } else {
        drive_motors.setM1Speed(pan_speed);
        drive_motors.setM2Speed(-pan_speed);
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

void StepperAndSensors::setDrive(int16_t left_speed, int16_t right_speed) {
    // If panning is active, ignore external drive commands to prevent conflict
    if (is_panning) return;

    // Using AStar32U4Motors library
    // Library range: -400 to 400
    // Python now sends full range as 16-bit signed integers

    // Clamp to motor library limits (-400 to 400)
    int16_t left_motor = constrain(left_speed, -400, 400);
    int16_t right_motor = constrain(right_speed, -400, 400);

    // M1 = Right motor, M2 = Left motor (typical A-Star configuration)
    drive_motors.setM1Speed(right_motor);
    drive_motors.setM2Speed(left_motor);
}
