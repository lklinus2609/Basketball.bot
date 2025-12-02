# Project Architecture & Conventions

## System Overview

The **Bevo Madness Basketball Robot** uses a distributed control architecture:
- **Raspberry Pi 4 (High-Level)**: Runs the main state machine, handles strategy, reads IR sensors directly, and manages serial communication.
- **Pololu A-Star 32U4 (Low-Level)**: Handles real-time motor control (steppers, drive motors) and executes commands from the Pi.

## Hardware Architecture

### Raspberry Pi 4
- **Language**: Python 3
- **OS**: Raspberry Pi OS
- **Connections**:
  - **GPIO 17**: IR Sensor Input (Active Low)
  - **USB/Serial**: Communication with A-Star (`/dev/ttyAMA0` or `/dev/ttyACM0`)

### Pololu A-Star 32U4
- **Language**: C++ (Arduino)
- **Connections**:
  - **A0-A3**: Encoders (Left/Right A/B)
  - **A4-A5**: Loader Stepper (STEP/DIR)
  - **D4, D8**: Ultrasonic Sensor (TRIG, ECHO)
  - **D12**: Loader Enable
  - **D6, D7, D5**: Left Drive Motor (IN1, IN2, PWM) - *Example mapping, check config.h*
  - **D10, D11, D9**: Right Drive Motor (IN1, IN2, PWM) - *Example mapping, check config.h*

## Software Architecture

### Directory Structure
```
bot_test/
├── main.py                  # Entry point
├── state_machine.py         # Main logic (7-state FSM)
├── communication.py         # Serial protocol implementation
├── ballistics.py            # RPM lookup and physics
├── config.py                # Global configuration parameters
├── astar_controller/        # Firmware for A-Star
│   ├── astar_controller.ino # Main firmware loop
│   ├── config.h             # Firmware pin definitions
│   ├── communication.h/.cpp # Firmware serial protocol
│   └── stepper_and_sensors.h/.cpp # Hardware drivers
└── IRscan/                  # Reference implementation for IR logic
```

### Communication Protocol
- **Type**: Asynchronous Serial (UART)
- **Baud Rate**: 115200
- **Format**: Binary Packets
  - `[START_BYTE][CMD_TYPE][DATA_LEN][DATA...][CHECKSUM]`
- **Key Commands**:
  - `CMD_SET_IR_STATE (0x20)`: Pi -> Arduino (0=Detected, 1=Not Detected)
  - `CMD_START_PANNING (0x70)`: Pi -> Arduino (Start differential drive rotation)
  - `CMD_SHOOT (0x30)`: Pi -> Arduino (Trigger loader)

### State Machine (Python)
1. **INIT**: Setup serial, GPIO.
2. **SEEK_AND_ORIENT**: (Currently simplified) Transitions to hunting.
3. **HUNT_FOR_TARGET**:
   - Sends `CMD_START_PANNING`.
   - Reads GPIO 17 (IR).
   - Sends `CMD_SET_IR_STATE` to Arduino.
   - Transitions when IR detected.
4. **CALCULATE_SHOT**: Look up RPM/Distance.
5. **ALIGN_AND_SPINUP**: Wait for stability.
6. **EXECUTE_SHOT**: Trigger loader.
7. **END_GAME**: Stop all motors.

## Coding Conventions

### Python
- **Style**: PEP 8 compliant.
- **Imports**: Grouped (Standard lib, Third-party, Local).
- **Constants**: UPPER_CASE in `config.py`.
- **Classes**: CamelCase (e.g., `BasketballRobotStateMachine`).

### C++ (Arduino)
- **Style**: Arduino standard.
- **Constants**: `#define` or `const` in `config.h`.
- **Classes**: CamelCase (e.g., `StepperAndSensors`).
- **Non-Blocking**: No `delay()` in main loop. Use `millis()` for timing.

## Deprecated Features
- **Lazy Susan**: Removed in favor of differential drive panning.
- **Flywheel Control (Firmware)**: Disabled in firmware; flywheels controlled externally.
- **Complex Localization**: 360° scan and triangulation temporarily disabled in favor of direct IR hunting.
