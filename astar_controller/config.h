/**
 * A-Star Controller Configuration
 * Pin definitions and constants for basketball robot
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// PIN DEFINITIONS (Adjust for your A-Star wiring)
// ============================================================================

// L298N Flywheel Motor Control
#define FLYWHEEL_LEFT_PWM       5   // PWM pin for left flywheel
#define FLYWHEEL_LEFT_IN1       6   // Direction pin 1
#define FLYWHEEL_LEFT_IN2       7   // Direction pin 2

#define FLYWHEEL_RIGHT_PWM      9   // PWM pin for right flywheel
#define FLYWHEEL_RIGHT_IN1      10  // Direction pin 1
#define FLYWHEEL_RIGHT_IN2      11  // Direction pin 2

// Flywheel Encoders (interrupt pins)
#define ENCODER_LEFT_A          2   // INT0
#define ENCODER_LEFT_B          4

#define ENCODER_RIGHT_A         3   // INT1
#define ENCODER_RIGHT_B          8

// DRV8825 Lazy Susan Stepper
#define LAZY_SUSAN_STEP         A0
#define LAZY_SUSAN_DIR          A1
#define LAZY_SUSAN_ENABLE       A2

// DRV8825 Loading Mechanism Stepper
#define LOADER_STEP             A3
#define LOADER_DIR              A4
#define LOADER_ENABLE           A5

// IR Sensors (TSOP34156 - active low) - Only 2 sensors
#define IR_SENSOR_LEFT          17
#define IR_SENSOR_RIGHT         18

// Ultrasonic Sensor (HC-SR04 or similar)
#define ULTRASONIC_TRIG         9
#define ULTRASONIC_ECHO         8

// Drive Wheels - Using AStar32U4Motors library (built-in motor drivers)
// M1 = Right motor, M2 = Left motor
// No pin definitions needed - library handles it internally

// ============================================================================
// MOTOR CONSTANTS
// ============================================================================

// Flywheel PID tuning parameters (TUNE THESE!)
#define FLYWHEEL_KP             0.5
#define FLYWHEEL_KI             0.1
#define FLYWHEEL_KD             0.05

#define PID_UPDATE_RATE_HZ      50      // 50 Hz PID updates
#define PID_UPDATE_INTERVAL_MS  (1000 / PID_UPDATE_RATE_HZ)

#define MAX_PWM                 255
#define MIN_PWM                 0

// Encoder pulses per revolution (adjust for your encoders!)
#define ENCODER_PPR             360     // Typical value - measure yours!

// Anti-windup limits for integral term
#define MAX_INTEGRAL            1000.0
#define MIN_INTEGRAL            -1000.0

// ============================================================================
// STEPPER CONSTANTS
// ============================================================================

// NEMA17 = 200 steps/rev
#define STEPS_PER_REV           200

// Microstepping settings (set DRV8825 jumpers accordingly!)
#define LAZY_SUSAN_MICROSTEPS   8       // 1/8 microstepping
#define LOADER_MICROSTEPS       16      // 1/16 microstepping

// Calculated steps per degree
#define LAZY_SUSAN_STEPS_PER_DEG  ((STEPS_PER_REV * LAZY_SUSAN_MICROSTEPS) / 360.0)
#define LOADER_STEPS_PER_90_DEG   ((STEPS_PER_REV * LOADER_MICROSTEPS) / 4)

// Lazy Susan motion parameters
#define LAZY_SUSAN_MAX_SPEED    800     // steps/second
#define LAZY_SUSAN_ACCELERATION 2000    // steps/s²

// Loader motion parameters (fast for shooting)
#define LOADER_MAX_SPEED        1000    // steps/second
#define LOADER_ACCELERATION     3000    // steps/s²

// ============================================================================
// SENSOR CONSTANTS
// ============================================================================

// Ultrasonic sensor
#define ULTRASONIC_TIMEOUT_US   15000   // 15ms timeout (~2.5m max range)
#define SOUND_SPEED_CM_US       0.0343  // Speed of sound: 343 m/s

// IR sensor debounce time
#define IR_DEBOUNCE_MS          50

// ============================================================================
// COMMUNICATION CONSTANTS
// ============================================================================

#define SERIAL_BAUD             115200
#define TELEMETRY_RATE_HZ       50      // 50 Hz telemetry updates
#define TELEMETRY_INTERVAL_MS   (1000 / TELEMETRY_RATE_HZ)

#define START_BYTE              0xFF
#define MAX_PACKET_SIZE         32

// Command types (must match Python code!)
#define CMD_SET_FLYWHEEL_RPM    0x10
#define CMD_ROTATE_LAZY_SUSAN   0x20
#define CMD_SHOOT               0x30
#define CMD_SET_DRIVE           0x40
#define CMD_RESET               0x50
#define CMD_GET_STATUS          0x60

// Telemetry types
#define TEL_SENSOR_DATA         0x80
#define TEL_FLYWHEEL_STATUS     0x81
#define TEL_POSITION_STATUS     0x82
#define TEL_LOADER_STATUS       0x83
#define TEL_DEBUG_MESSAGE       0x8F

// ============================================================================
// DEBUG SETTINGS
// ============================================================================

#define DEBUG_MODE              1       // Set to 0 to disable debug messages
#define DEBUG_PRINT_INTERVAL_MS 1000    // Print debug info every 1 second

#endif // CONFIG_H
