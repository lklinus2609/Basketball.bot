/**
 * A-Star Basketball Robot Controller
 * ME 348E/392Q Advanced Mechatronics - Fall 2025
 *
 * Main control loop for A-Star
 * Handles low-level control of:
 *  - Dual flywheel motors (PID control)
 *  - Lazy susan stepper motor
 *  - Loading mechanism stepper motor
 *  - IR sensors (TSOP34156)
 *  - Ultrasonic sensor
 *  - Encoders (for panning)
 *
 * Communicates with Raspberry Pi via serial
 */

#include "config.h"
#include "communication.h"
#include "flywheel_control.h"
#include "stepper_and_sensors.h"
#include <Encoder.h>

// Encoders: Right wheel = A0, A1 | Left wheel = A2, A3
// Resolution: 1440 counts per wheel revolution
Encoder encoderRight(A0, A1);  // Right wheel encoder
Encoder encoderLeft(A2, A3);   // Left wheel encoder

// Timing
unsigned long last_telemetry_time = 0;
unsigned long last_debug_time = 0;

void setup() {
    // Initialize communication
    comm.begin();

    // Wait for serial connection
    delay(1000);

    #if DEBUG_MODE
    comm.sendDebugMessage("A-Star initializing...");
    #endif

    // Initialize flywheel controller
    flywheels.begin();

    // Initialize steppers and sensors (pass encoder references)
    steppers_sensors.begin(&encoderLeft, &encoderRight);

    #if DEBUG_MODE
    comm.sendDebugMessage("A-Star ready!");
    #endif

    last_telemetry_time = millis();
    last_debug_time = millis();
}

void loop() {
    unsigned long current_time = millis();

    // ========================================================================
    // COMMUNICATION UPDATE (non-blocking)
    // ========================================================================
    comm.update();

    // Process received commands
    if (comm.hasPacket()) {
        CommandData cmd;
        comm.parsePacket(cmd);
        handleCommand(cmd);
    }

    // ========================================================================
    // CONTROLLER UPDATES (non-blocking)
    // ========================================================================

    // Update flywheel PID (runs at fixed rate internally)
    flywheels.update();

    // Update steppers and sensors (includes panning logic)
    steppers_sensors.update();

    // ========================================================================
    // TELEMETRY TRANSMISSION (at fixed rate)
    // ========================================================================
    if (current_time - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
        sendTelemetry();
        last_telemetry_time = current_time;
    }

    // ========================================================================
    // DEBUG OUTPUT (optional, periodic)
    // ========================================================================
    #if DEBUG_MODE
    if (current_time - last_debug_time >= DEBUG_PRINT_INTERVAL_MS) {
        printDebugInfo();
        last_debug_time = current_time;
    }
    #endif
}

void handleCommand(CommandData& cmd) {
    switch (cmd.type) {
        case CMD_SET_FLYWHEEL_RPM:
            flywheels.setTargetRPM(cmd.flywheel_rpm);
            break;

        case CMD_ROTATE_LAZY_SUSAN:
            steppers_sensors.rotateLazySusan(cmd.lazy_susan_angle);
            break;

        case CMD_SHOOT:
            steppers_sensors.shoot();
            #if DEBUG_MODE
            comm.sendDebugMessage("SHOOT!");
            #endif
            break;

        case CMD_SET_DRIVE:
            steppers_sensors.setDrive(cmd.drive_left, cmd.drive_right);
            break;
            
        case CMD_START_PANNING:
            steppers_sensors.startPanning(cmd.panning_speed);
            #if DEBUG_MODE
            comm.sendDebugMessage("START PANNING");
            #endif
            break;
            
        case CMD_STOP_PANNING:
            steppers_sensors.stopPanning();
            #if DEBUG_MODE
            comm.sendDebugMessage("STOP PANNING");
            #endif
            break;

        case CMD_RESET:
            // Software reset
            flywheels.stop();
            steppers_sensors.rotateLazySusan(0);
            steppers_sensors.stopPanning();
            #if DEBUG_MODE
            comm.sendDebugMessage("Reset");
            #endif
            break;
    }
}

void sendTelemetry() {
    // Send sensor data (only 2 IR sensors)
    uint8_t ir_left = steppers_sensors.readIR_Left();
    uint8_t ir_right = steppers_sensors.readIR_Right();
    uint16_t distance = steppers_sensors.readUltrasonic();

    comm.sendSensorData(ir_left, ir_right, distance);

    // Send flywheel status
    uint16_t rpm_left = flywheels.getRPM_Left();
    uint16_t rpm_right = flywheels.getRPM_Right();

    comm.sendFlywheelStatus(rpm_left, rpm_right);

    // Send position status
    float angle = steppers_sensors.getLazySusanAngle();
    comm.sendPositionStatus(angle);

    // Send loader status
    bool loader_ready = steppers_sensors.isLoaderReady();
    comm.sendLoaderStatus(loader_ready);
}

void printDebugInfo() {
    // Print periodic debug information (not sent to RPI, just for serial monitor)
    // Comment out if not needed
}
