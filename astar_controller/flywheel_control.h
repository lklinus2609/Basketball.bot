/**
 * Flywheel PID Controller Header
 */

#ifndef FLYWHEEL_CONTROL_H
#define FLYWHEEL_CONTROL_H

#include <Arduino.h>

class FlywheelController {
public:
    FlywheelController();
    void begin();
    void update();

    void setTargetRPM(uint16_t rpm);
    uint16_t getRPM_Left();
    uint16_t getRPM_Right();
    void stop();

private:
    uint16_t target_rpm_left;
    uint16_t target_rpm_right;

    float current_rpm_left;
    float current_rpm_right;

    // PID state
    float integral_left;
    float integral_right;
    float prev_error_left;
    float prev_error_right;

    // Timing
    unsigned long last_update_time;
    unsigned long last_rpm_calc_time;

    // Encoder tracking
    long prev_encoder_left;
    long prev_encoder_right;

    void calculateRPM();
};

extern FlywheelController flywheels;

#endif // FLYWHEEL_CONTROL_H
