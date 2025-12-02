# Testing Guide

This guide outlines the steps to verify the robot's functionality, from low-level hardware checks to full autonomous operation.

## 1. Hardware Verification

Before running the full state machine, verify each component individually.

### A. IR Sensor (Raspberry Pi)
1.  Connect the IR sensor to **GPIO 17**.
2.  Run a simple Python script to check readings:
    ```python
    import RPi.GPIO as GPIO
    import time

    PIN = 17
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    try:
        while True:
            state = GPIO.input(PIN)
            print(f"IR State: {state} (0=Detected, 1=Clear)")
            time.sleep(0.1)
    except KeyboardInterrupt:
        GPIO.cleanup()
    ```
3.  Shine an IR beacon at the sensor and confirm the state changes to `0`.

### B. Communication & Motors (A-Star)
1.  Ensure the A-Star firmware is uploaded.
2.  Run the communication test:
    ```bash
    python3 communication.py
    ```
    *(Note: You may need to uncomment the `if __name__ == "__main__":` block in `communication.py` if it was removed, or create a small test script)*
3.  **Test Panning**:
    ```python
    from communication import AStarCommunication
    import time
    
    comm = AStarCommunication()
    comm.connect()
    comm.start_panning(75)
    time.sleep(2)
    comm.stop_panning()
    ```
    *Robot should rotate.*

4.  **Test Loader**:
    ```python
    comm.shoot()
    ```
    *Loader should rotate 90 degrees.*

## 2. State Machine Logic Test (Dry Run)

You can test the logic without full hardware by mocking the communication or just observing the logs.

1.  **Enable Debug Mode**: Ensure `DEBUG_MODE = True` in `config.py`.
2.  **Run Main**:
    ```bash
    python3 main.py
    ```
3.  **Observe Output**:
    - It should print `[INIT] ...`.
    - It should enter `HUNT_FOR_TARGET`.
    - It will print `[STATE] Starting PANNING mode...`.
4.  **Simulate IR**:
    - If you have the IR sensor connected, trigger it.
    - The state should change to `CALCULATE_SHOT`, then `ALIGN_AND_SPINUP`, then `EXECUTE_SHOT`.

## 3. Full System Test

1.  Place the robot on the court (or a safe test area).
2.  Load balls (or test empty).
3.  Run `python3 main.py`.
4.  Press `1` to start.
5.  **Expected Behavior**:
    - Robot starts rotating (Panning).
    - When IR beacon is active/detected:
        - Robot stops immediately.
        - Loader fires (clicks/rotates).
        - Robot resumes panning (if more balls remain).

## Troubleshooting

-   **Robot doesn't stop on IR**: Check GPIO 17 connection and `CMD_SET_IR_STATE` logic.
-   **Robot rotates forever**: Ensure `start_panning` is sending the correct speed and motors are powered.
-   **Loader doesn't fire**: Check `CMD_SHOOT` and stepper wiring.
