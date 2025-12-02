# Basketball Robot - Bevo Madness

Autonomous basketball robot for ME 348E/392Q Fall 2025 competition.

## 🚀 Quick Start

### 1. Hardware Setup
- **Raspberry Pi**: Connect IR sensor to GPIO 17.
- **A-Star**: Connect Encoders (A0-A3), Loader (A4-A5), Ultrasonic (4, 8).
- **Power**: Ensure 12V supply for motors and 5V for logic.

### 2. Upload Firmware
1. Open `astar_controller/astar_controller.ino` in Arduino IDE.
2. Install `AccelStepper` library.
3. Upload to Pololu A-Star 32U4.

### 3. Run Robot
```bash
# SSH into Raspberry Pi
cd ~/bot_test
python3 main.py
# Press ENTER to start
```

## 🤖 How It Works

### 1. Hunting (Panning)
- The robot rotates in place using differential drive (`start_panning`).
- The Raspberry Pi reads the IR sensor (GPIO 17).
- When the beacon is detected, the Pi sends a signal (`set_ir_state`) to the Arduino.
- The Arduino immediately stops the motors.

### 2. Shooting
- Once stopped, the robot calculates the shot parameters (Distance lookup).
- It triggers the loading mechanism (`shoot`).
- The loader rotates 90° to feed a ball into the flywheels.
- **Note**: Flywheels are controlled by an external microcontroller.

### 3. Cycle
- The robot repeats this cycle for all 10 balls.

## 📁 Project Structure

- `main.py`: Entry point.
- `state_machine.py`: Main logic and state management.
- `communication.py`: Serial communication with A-Star.
- `config.py`: Configuration parameters.
- `astar_controller/`: Arduino firmware.
- `ARCHITECTURE.md`: Detailed system architecture and conventions.

## ⚠️ Important Notes

- **Flywheel Control**: The firmware code for flywheel control is disabled. Ensure the external controller is running.
- **Lazy Susan**: The "Lazy Susan" mechanism is no longer used for panning; the entire robot rotates.
