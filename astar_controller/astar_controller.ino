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
#include "stepper_and_sensors.h"
#include <Encoder.h>

// Encoders: Right wheel = A0, A1 | Left wheel = A2, A3
// Resolution: 1440 counts per wheel revolution
Encoder encoderRight(ENCODER_RIGHT_A, ENCODER_RIGHT_B);
Encoder encoderLeft(ENCODER_LEFT_A, ENCODER_LEFT_B);

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
            // Flywheel controlled externally - do nothing
            break;

        case CMD_SET_IR_STATE:
            steppers_sensors.setIRState(cmd.ir_state);
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
            
        case CMD_RESET:
            // Software reset
            steppers_sensors.stopPanning();
            #if DEBUG_MODE
            comm.sendDebugMessage("Reset");
            #endif
            break;
    }
}

void sendTelemetry() {
    // Send sensor data (only 2 IR sensors + Ultrasonic)
    // Note: IR sensors are now read by Pi, but we can send raw values if needed
    // For now, sending 0s or raw reads for debug
    uint8_t ir_left = 0; 
    uint8_t ir_right = 0;
    uint16_t distance = steppers_sensors.readUltrasonic();

    comm.sendSensorData(ir_left, ir_right, distance);

    // Send loader status
    bool loader_ready = steppers_sensors.isLoaderReady();
    comm.sendLoaderStatus(loader_ready);
}

void printDebugInfo() {
    // Print periodic debug information (not sent to RPI, just for serial monitor)
    // Comment out if not needed
}
