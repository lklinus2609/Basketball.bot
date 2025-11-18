/**
 * Dual flywheel PID control
 * Implements non-blocking PID controller for both flywheels
 */

#include "flywheel_control.h"
#include "config.h"

FlywheelController flywheels;

// Encoder pulse counters (updated by ISR)
volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;

// Interrupt service routines for encoders
void encoderLeftISR() {
    encoder_left_count++;
}

void encoderRightISR() {
    encoder_right_count++;
}

FlywheelController::FlywheelController() {
    target_rpm_left = 0;
    target_rpm_right = 0;
    current_rpm_left = 0;
    current_rpm_right = 0;

    integral_left = 0;
    integral_right = 0;
    prev_error_left = 0;
    prev_error_right = 0;

    last_update_time = 0;
    last_rpm_calc_time = 0;
    prev_encoder_left = 0;
    prev_encoder_right = 0;
}

void FlywheelController::begin() {
    // Setup motor control pins
    pinMode(FLYWHEEL_LEFT_PWM, OUTPUT);
    pinMode(FLYWHEEL_LEFT_IN1, OUTPUT);
    pinMode(FLYWHEEL_LEFT_IN2, OUTPUT);

    pinMode(FLYWHEEL_RIGHT_PWM, OUTPUT);
    pinMode(FLYWHEEL_RIGHT_IN1, OUTPUT);
    pinMode(FLYWHEEL_RIGHT_IN2, OUTPUT);

    // Setup encoder pins
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

    // Attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoderLeftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoderRightISR, RISING);

    // Set motor direction (forward for both)
    digitalWrite(FLYWHEEL_LEFT_IN1, HIGH);
    digitalWrite(FLYWHEEL_LEFT_IN2, LOW);
    digitalWrite(FLYWHEEL_RIGHT_IN1, HIGH);
    digitalWrite(FLYWHEEL_RIGHT_IN2, LOW);

    // Initialize PWM to 0
    analogWrite(FLYWHEEL_LEFT_PWM, 0);
    analogWrite(FLYWHEEL_RIGHT_PWM, 0);

    last_update_time = millis();
    last_rpm_calc_time = millis();
}

void FlywheelController::setTargetRPM(uint16_t rpm) {
    target_rpm_left = rpm;
    target_rpm_right = rpm;
}

void FlywheelController::calculateRPM() {
    unsigned long current_time = millis();
    float dt_sec = (current_time - last_rpm_calc_time) / 1000.0;

    if (dt_sec >= 0.1) { // Calculate RPM every 100ms
        // Read encoder counts (atomic read)
        noInterrupts();
        long count_left = encoder_left_count;
        long count_right = encoder_right_count;
        interrupts();

        // Calculate pulses since last reading
        long delta_left = count_left - prev_encoder_left;
        long delta_right = count_right - prev_encoder_right;

        // Calculate RPM: (pulses / dt) * (60 / PPR)
        current_rpm_left = (delta_left / dt_sec) * (60.0 / ENCODER_PPR);
        current_rpm_right = (delta_right / dt_sec) * (60.0 / ENCODER_PPR);

        // Update previous values
        prev_encoder_left = count_left;
        prev_encoder_right = count_right;
        last_rpm_calc_time = current_time;
    }
}

void FlywheelController::update() {
    unsigned long current_time = millis();

    // Update RPM calculation
    calculateRPM();

    // PID update at fixed rate
    if (current_time - last_update_time >= PID_UPDATE_INTERVAL_MS) {
        float dt = (current_time - last_update_time) / 1000.0; // seconds

        // PID for left flywheel
        float error_left = target_rpm_left - current_rpm_left;

        integral_left += error_left * dt;
        integral_left = constrain(integral_left, MIN_INTEGRAL, MAX_INTEGRAL);

        float derivative_left = (error_left - prev_error_left) / dt;

        float output_left = FLYWHEEL_KP * error_left +
                            FLYWHEEL_KI * integral_left +
                            FLYWHEEL_KD * derivative_left;

        // PID for right flywheel
        float error_right = target_rpm_right - current_rpm_right;

        integral_right += error_right * dt;
        integral_right = constrain(integral_right, MIN_INTEGRAL, MAX_INTEGRAL);

        float derivative_right = (error_right - prev_error_right) / dt;

        float output_right = FLYWHEEL_KP * error_right +
                             FLYWHEEL_KI * integral_right +
                             FLYWHEEL_KD * derivative_right;

        // Apply PWM
        int pwm_left = constrain((int)output_left, MIN_PWM, MAX_PWM);
        int pwm_right = constrain((int)output_right, MIN_PWM, MAX_PWM);

        analogWrite(FLYWHEEL_LEFT_PWM, pwm_left);
        analogWrite(FLYWHEEL_RIGHT_PWM, pwm_right);

        // Update previous errors
        prev_error_left = error_left;
        prev_error_right = error_right;

        last_update_time = current_time;
    }
}

uint16_t FlywheelController::getRPM_Left() {
    return (uint16_t)current_rpm_left;
}

uint16_t FlywheelController::getRPM_Right() {
    return (uint16_t)current_rpm_right;
}

void FlywheelController::stop() {
    setTargetRPM(0);
}
