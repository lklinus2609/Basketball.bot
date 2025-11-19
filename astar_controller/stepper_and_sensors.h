/**
 * Stepper and Sensors Header
 */

#ifndef STEPPER_AND_SENSORS_H
#define STEPPER_AND_SENSORS_H

#include <Arduino.h>

class StepperAndSensors {
public:
    StepperAndSensors();
    void begin();
    void update();

    // Lazy susan control
    void rotateLazySusan(float angle_degrees);
    float getLazySusanAngle();

    // Loader control
    void shoot();
    bool isLoaderReady();

    // Sensor reading (only 2 IR sensors)
    uint8_t readIR_Left();
    uint8_t readIR_Right();
    uint16_t readUltrasonic();

    // Drive wheel control
    void setDrive(int8_t left_speed, int8_t right_speed);

private:
    bool loader_ready;
    float current_lazy_susan_angle;
};

extern StepperAndSensors steppers_sensors;

#endif // STEPPER_AND_SENSORS_H
