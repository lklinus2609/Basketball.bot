/**
 * Stepper and Sensors Header
 */

#ifndef STEPPER_AND_SENSORS_H
#define STEPPER_AND_SENSORS_H

#include <Arduino.h>
#include <Encoder.h>

class StepperAndSensors {
public:
    StepperAndSensors();
    void begin(Encoder* left, Encoder* right);
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

    // Drive wheel control (supports full -400 to +400 motor range)
    void setDrive(int16_t left_speed, int16_t right_speed);
    
    // Panning control
    void startPanning(int16_t speed);
    void stopPanning();

private:
    bool loader_ready;
    float current_lazy_susan_angle;
    
    // Panning state
    bool is_panning;
    int16_t pan_speed;
    int8_t pan_direction; // 1 = right, -1 = left
    long pan_rotation_target;
    
    // Encoder references
    Encoder* encoder_left;
    Encoder* encoder_right;
    
    void updatePanning();
};

extern StepperAndSensors steppers_sensors;

#endif // STEPPER_AND_SENSORS_H
