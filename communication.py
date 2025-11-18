"""
Non-blocking serial communication with A-Star controller
Implements packet-based protocol with checksums
"""

import serial
import struct
import time
from enum import IntEnum
from config import SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT, DEBUG_MODE


class CommandType(IntEnum):
    """Command types sent from RPI to A-Star"""
    SET_FLYWHEEL_RPM = 0x10
    ROTATE_LAZY_SUSAN = 0x20
    SHOOT = 0x30
    SET_DRIVE = 0x40
    RESET = 0x50
    GET_STATUS = 0x60


class TelemetryType(IntEnum):
    """Telemetry types received from A-Star"""
    SENSOR_DATA = 0x80
    FLYWHEEL_STATUS = 0x81
    POSITION_STATUS = 0x82
    LOADER_STATUS = 0x83
    DEBUG_MESSAGE = 0x8F


class AStarCommunication:
    """Handles non-blocking serial communication with A-Star"""

    START_BYTE = 0xFF
    PACKET_OVERHEAD = 4  # START + CMD + LEN + CHECKSUM

    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
        """Initialize serial connection"""
        self.port = port
        self.baud = baud
        self.serial = None
        self.rx_buffer = bytearray()

        # Latest telemetry data
        self.telemetry = {
            'ir_left': 0,
            'ir_center': 0,
            'ir_right': 0,
            'distance_cm': 0,
            'flywheel_rpm_left': 0,
            'flywheel_rpm_right': 0,
            'lazy_susan_angle': 0.0,
            'loader_ready': True,
            'timestamp': 0
        }

        self.connected = False

    def connect(self):
        """Open serial connection"""
        try:
            self.serial = serial.Serial(
                self.port,
                self.baud,
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_TIMEOUT
            )
            time.sleep(0.1)  # Wait for connection to stabilize
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

    def _calculate_checksum(self, data):
        """Calculate simple checksum (sum of bytes & 0xFF)"""
        return sum(data) & 0xFF

    def send_command(self, cmd_type, data=None):
        """
        Send command packet to A-Star

        Packet format: [START_BYTE][CMD][LEN][DATA...][CHECKSUM]

        Args:
            cmd_type: CommandType enum value
            data: Optional bytes to send
        """
        if not self.connected or not self.serial:
            return False

        try:
            if data is None:
                data = b''

            packet = bytearray()
            packet.append(self.START_BYTE)
            packet.append(cmd_type)
            packet.append(len(data))
            packet.extend(data)

            checksum = self._calculate_checksum(packet[1:])  # Exclude START_BYTE
            packet.append(checksum)

            self.serial.write(packet)
            return True

        except Exception as e:
            if DEBUG_MODE:
                print(f"[COMM] Send error: {e}")
            return False

    def set_flywheel_rpm(self, rpm):
        """
        Set target RPM for both flywheels

        Args:
            rpm: Target RPM (0-10000)
        """
        # Pack as unsigned 16-bit integer
        data = struct.pack('<H', int(rpm))
        return self.send_command(CommandType.SET_FLYWHEEL_RPM, data)

    def rotate_lazy_susan(self, angle):
        """
        Rotate lazy susan to absolute angle

        Args:
            angle: Target angle in degrees (-180 to +180)
        """
        # Pack as signed 16-bit integer (angle * 10 for 0.1° resolution)
        angle_encoded = int(angle * 10)
        data = struct.pack('<h', angle_encoded)
        return self.send_command(CommandType.ROTATE_LAZY_SUSAN, data)

    def shoot(self):
        """Trigger shooting mechanism (90° rotation)"""
        return self.send_command(CommandType.SHOOT)

    def set_drive(self, left_speed, right_speed):
        """
        Set drive wheel speeds

        Args:
            left_speed: Left wheel speed (-255 to +255)
            right_speed: Right wheel speed (-255 to +255)
        """
        data = struct.pack('<bb', int(left_speed), int(right_speed))
        return self.send_command(CommandType.SET_DRIVE, data)

    def reset(self):
        """Reset A-Star controller"""
        return self.send_command(CommandType.RESET)

    def update(self):
        """
        Non-blocking update - read and process incoming telemetry
        Call this frequently (e.g., every main loop iteration)
        """
        if not self.connected or not self.serial:
            return

        try:
            # Read available bytes (non-blocking)
            if self.serial.in_waiting > 0:
                incoming = self.serial.read(self.serial.in_waiting)
                self.rx_buffer.extend(incoming)

            # Process complete packets
            while len(self.rx_buffer) >= self.PACKET_OVERHEAD:
                # Find start byte
                if self.rx_buffer[0] != self.START_BYTE:
                    # Invalid, discard byte and continue
                    self.rx_buffer.pop(0)
                    continue

                # Check if we have enough bytes for header
                if len(self.rx_buffer) < 3:
                    break

                msg_type = self.rx_buffer[1]
                data_len = self.rx_buffer[2]

                # Check if complete packet available
                packet_len = 3 + data_len + 1  # Header + data + checksum
                if len(self.rx_buffer) < packet_len:
                    break  # Wait for more data

                # Extract packet
                packet = self.rx_buffer[:packet_len]
                self.rx_buffer = self.rx_buffer[packet_len:]

                # Verify checksum
                received_checksum = packet[-1]
                calculated_checksum = self._calculate_checksum(packet[1:-1])

                if received_checksum != calculated_checksum:
                    if DEBUG_MODE:
                        print(f"[COMM] Checksum error: {received_checksum} != {calculated_checksum}")
                    continue

                # Parse telemetry
                data = packet[3:-1]
                self._parse_telemetry(msg_type, data)

        except Exception as e:
            if DEBUG_MODE:
                print(f"[COMM] Update error: {e}")

    def _parse_telemetry(self, msg_type, data):
        """Parse incoming telemetry packet"""
        try:
            if msg_type == TelemetryType.SENSOR_DATA:
                # IR sensors (3 bytes) + ultrasonic distance (2 bytes)
                if len(data) >= 5:
                    self.telemetry['ir_left'] = data[0]
                    self.telemetry['ir_center'] = data[1]
                    self.telemetry['ir_right'] = data[2]
                    self.telemetry['distance_cm'] = struct.unpack('<H', data[3:5])[0]
                    self.telemetry['timestamp'] = time.time()

            elif msg_type == TelemetryType.FLYWHEEL_STATUS:
                # Both flywheel RPMs (2x 2 bytes)
                if len(data) >= 4:
                    self.telemetry['flywheel_rpm_left'] = struct.unpack('<H', data[0:2])[0]
                    self.telemetry['flywheel_rpm_right'] = struct.unpack('<H', data[2:4])[0]

            elif msg_type == TelemetryType.POSITION_STATUS:
                # Lazy susan angle (2 bytes, signed, 0.1° resolution)
                if len(data) >= 2:
                    angle_encoded = struct.unpack('<h', data[0:2])[0]
                    self.telemetry['lazy_susan_angle'] = angle_encoded / 10.0

            elif msg_type == TelemetryType.LOADER_STATUS:
                # Loader ready flag (1 byte)
                if len(data) >= 1:
                    self.telemetry['loader_ready'] = bool(data[0])

            elif msg_type == TelemetryType.DEBUG_MESSAGE:
                # ASCII debug message
                msg = data.decode('ascii', errors='ignore')
                print(f"[A-STAR DEBUG] {msg}")

        except Exception as e:
            if DEBUG_MODE:
                print(f"[COMM] Parse error: {e}")

    def get_active_beacon(self):
        """
        Determine which IR beacon is active

        Returns:
            'LEFT', 'CENTER', 'RIGHT', or None
        """
        # TSOP34156 is active-low: LOW (0) = beacon detected
        ir_left = self.telemetry['ir_left'] == 0
        ir_center = self.telemetry['ir_center'] == 0
        ir_right = self.telemetry['ir_right'] == 0

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
