"""
Non-blocking serial communication with A-Star controller
Implements packet-based protocol with checksums
"""

import serial
import struct
import time
from enum import IntEnum
from collections import deque
from config import (
    SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT, DEBUG_MODE,
    IR_FILTER_SAMPLES, IR_FILTER_THRESHOLD_HIGH, IR_FILTER_THRESHOLD_LOW
)


class AStarCommunication:
    """Handles serial communication with A-Star using ASCII protocol"""

    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
        """Initialize serial connection"""
        self.port = port
        self.baud = baud
        self.serial = None
        self.connected = False
        
        # State to send
        self.cmd_speed = 0
        self.cmd_ir_state = 1 # 1 = Not Detected
        self.cmd_shoot = 0    # 1 = Trigger

        # Telemetry (Placeholder)
        self.telemetry = {
            'loader_ready': True,
            'distance_cm': 0
        }

    def connect(self):
        """Open serial connection"""
        try:
            self.serial = serial.Serial(
                self.port,
                self.baud,
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_TIMEOUT
            )
            time.sleep(2)  # Wait for Arduino reset
            self.connected = True
            if DEBUG_MODE:
                print(f"[COMM] Connected to A-Star on {self.port}")
            return True
        except Exception as e:
            print(f"[COMM] ERROR: Failed to connect - {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False

    def _send_update(self):
        """Send current state to Arduino: <speed,ir,shoot>"""
        if not self.connected or not self.serial:
            return

        try:
            # Format: <speed,ir,shoot>
            command = f"<{self.cmd_speed},{self.cmd_ir_state},{self.cmd_shoot}>"
            self.serial.write(command.encode())
            
            # Reset one-shot triggers
            self.cmd_shoot = 0
            
        except Exception as e:
            print(f"[COMM] Send error: {e}")

    def set_flywheel_rpm(self, rpm):
        """No-op (Flywheel external)"""
        pass

    def set_ir_state(self, is_detected):
        """Update IR state"""
        self.cmd_ir_state = 0 if is_detected else 1
        self._send_update()

    def shoot(self):
        """Trigger shot"""
        self.cmd_shoot = 1
        self._send_update()

    def start_panning(self, speed):
        """Start panning"""
        self.cmd_speed = int(speed)
        self._send_update()

    def stop_panning(self):
        """Stop panning"""
        self.cmd_speed = 0
        self._send_update()
        
    def set_drive(self, left, right):
        """Legacy drive support (mapped to panning speed if possible)"""
        # Simple mapping: if turning, set pan speed
        if left != right:
            self.cmd_speed = abs(left)
        else:
            self.cmd_speed = 0
        self._send_update()

    def is_loader_ready(self):
        """Assume ready for now (no feedback in simple protocol)"""
        return True

    def get_distance_inches(self):
        """Placeholder"""
        return 0

    def update(self):
        """Read debug output from Arduino"""
        if not self.connected or not self.serial:
            return

        try:
            while self.serial.in_waiting > 0:
                line = self.serial.readline().decode('ascii', errors='ignore').strip()
                if line and DEBUG_MODE:
                    print(f"[ARDUINO] {line}")
        except Exception:
            pass

    def _update_ir_filter(self, sensor_name, raw_value):
        """
        Update hysteresis filter for a specific sensor
        raw_value: 0 (Detected) or 1 (Not Detected)
        """
        # Add to history (0 = Detected, 1 = Not Detected)
        # We want to count DETECTIONS, so we store True if raw_value == 0
        is_detected = (raw_value == 0)
        self.ir_history[sensor_name].append(is_detected)
        
        # Count detections in history
        detections = sum(self.ir_history[sensor_name])
        
        # Hysteresis Logic
        if self.ir_filtered_state[sensor_name]:
            # Currently DETECTED (ON)
            # Turn OFF only if detections drop below LOW threshold
            if detections < IR_FILTER_THRESHOLD_LOW:
                self.ir_filtered_state[sensor_name] = False
        else:
            # Currently NOT DETECTED (OFF)
            # Turn ON only if detections exceed HIGH threshold
            if detections >= IR_FILTER_THRESHOLD_HIGH:
                self.ir_filtered_state[sensor_name] = True

    def get_active_beacon(self):
        """
        Determine which IR beacon is active

        Returns:
            'LEFT', 'CENTER', 'RIGHT', or None
        """
        # Use FILTERED state instead of raw telemetry
        ir_left = self.ir_filtered_state['ir_left']
        ir_center = self.ir_filtered_state['ir_center']
        ir_right = self.ir_filtered_state['ir_right']

        # Return strongest signal (or first detected)
        if ir_center:
            return 'CENTER'
        elif ir_left:
            return 'LEFT'
        elif ir_right:
            return 'RIGHT'
        else:
            return None

    def get_distance_inches(self):
        """Get ultrasonic distance in inches"""
        cm = self.telemetry['distance_cm']
        return cm / 2.54

    def is_aligned(self, target_angle, tolerance=3.0):
        """Check if lazy susan is aligned to target"""
        current = self.telemetry['lazy_susan_angle']
        return abs(current - target_angle) < tolerance

    def is_flywheel_ready(self, target_rpm, tolerance_percent=3.0):
        """Check if flywheels are at target speed"""
        current = (self.telemetry['flywheel_rpm_left'] +
                   self.telemetry['flywheel_rpm_right']) / 2

        # Avoid division by zero when target_rpm is 0
        if target_rpm == 0:
            return current < 100  # Consider ready if stopped (< 100 RPM)

        error_percent = abs(current - target_rpm) / target_rpm * 100
        return error_percent < tolerance_percent

    def is_loader_ready(self):
        """Check if loader is ready to shoot"""
        return self.telemetry['loader_ready']


if __name__ == "__main__":
    # Test communication
    print("=== A-Star Communication Test ===")

    comm = AStarCommunication()

    if comm.connect():
        print("Connected! Sending test commands...")

        # Test commands
        comm.set_flywheel_rpm(5000)
        time.sleep(0.1)
        comm.rotate_lazy_susan(15.5)
        time.sleep(0.1)

        # Read telemetry for 2 seconds
        start = time.time()
        while time.time() - start < 2.0:
            comm.update()
            time.sleep(0.02)

            if time.time() - start > 1.0:  # Print after 1 sec
                print(f"\nTelemetry: {comm.telemetry}")
                break

        comm.disconnect()
    else:
        print("Connection failed!")
