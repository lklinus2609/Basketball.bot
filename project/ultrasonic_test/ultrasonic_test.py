#!/usr/bin/env python3
"""
Ultrasonic 360° Scan Test - Python Interface
Interacts with Arduino over serial to perform ultrasonic scans
Run this via SSH to control the robot remotely
"""

import serial
import time
import sys
import argparse

class UltrasonicTest:
    def __init__(self, port='/dev/ttyACM1', baud=115200):
        """Initialize serial connection to Arduino"""
        self.port = port
        self.baud = baud
        self.serial = None
        
    def connect(self):
        """Connect to the Arduino"""
        try:
            print(f"Connecting to {self.port} at {self.baud} baud...")
            self.serial = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            
            # Clear any startup messages
            while self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                print(line)
            
            print("\nConnected successfully!\n")
            return True
            
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            print("\nTry: ls /dev/ttyACM* or /dev/ttyUSB*")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected.")
    
    def send_command(self, cmd):
        """Send a command to the Arduino"""
        if self.serial and self.serial.is_open:
            self.serial.write(cmd.encode())
            self.serial.flush()
    
    def read_response(self, timeout=30):
        """Read response from Arduino until scan completes"""
        start_time = time.time()
        scan_data = []
        
        while time.time() - start_time < timeout:
            if self.serial.in_waiting:
                try:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(line)
                        
                        # Parse scan data (format: "Angle X°: Y cm")
                        if "Angle" in line and "°:" in line and "cm" in line:
                            try:
                                parts = line.split()
                                angle = int(parts[1].replace("°:", ""))
                                distance = int(parts[2])
                                scan_data.append((angle, distance))
                            except (ValueError, IndexError):
                                pass
                        
                        # Check if scan is complete
                        if "Ready for next command" in line or "Scan Complete" in line:
                            return scan_data
                            
                except UnicodeDecodeError:
                    pass
            
            time.sleep(0.01)
        
        return scan_data
    
    def start_scan(self):
        """Start a 360° ultrasonic scan"""
        print("\n" + "="*50)
        print("Starting 360° Ultrasonic Scan")
        print("="*50 + "\n")
        
        self.send_command('s')
        scan_data = self.read_response(timeout=60)
        
        if scan_data:
            print("\n" + "="*50)
            print("Scan Results Summary")
            print("="*50)
            
            # Find furthest wall
            max_distance = max(scan_data, key=lambda x: x[1])
            print(f"\nFurthest wall detected at:")
            print(f"  Angle: {max_distance[0]}°")
            print(f"  Distance: {max_distance[1]} cm")
            
            # Find closest wall
            min_distance = min(scan_data, key=lambda x: x[1])
            print(f"\nClosest wall detected at:")
            print(f"  Angle: {min_distance[0]}°")
            print(f"  Distance: {min_distance[1]} cm")
            print()
            
            return scan_data
        else:
            print("\nNo scan data received. Check Arduino connection.")
            return None
    
    def test_single_reading(self):
        """Test a single ultrasonic reading"""
        print("\nTesting single ultrasonic reading...")
        self.send_command('t')
        self.read_response(timeout=5)
    
    def calibrate_rotation(self):
        """Run rotation calibration"""
        print("\n" + "="*50)
        print("Rotation Calibration Mode")
        print("="*50)
        print("\nThe robot will rotate for 1 second.")
        print("Measure the angle and adjust ROTATION_TIME_PER_10DEG in the Arduino code.\n")
        
        input("Press Enter to start calibration...")
        
        self.send_command('c')
        self.read_response(timeout=10)
    
    def interactive_mode(self):
        """Run interactive command mode"""
        print("\n" + "="*50)
        print("Interactive Mode")
        print("="*50)
        print("\nCommands:")
        print("  s - Start 360° scan")
        print("  t - Test single ultrasonic reading")
        print("  c - Calibrate rotation timing")
        print("  q - Quit")
        print("="*50 + "\n")
        
        while True:
            try:
                cmd = input("Enter command (s/t/c/q): ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 's':
                    self.start_scan()
                elif cmd == 't':
                    self.test_single_reading()
                elif cmd == 'c':
                    self.calibrate_rotation()
                else:
                    print("Invalid command. Use s, t, c, or q")
                    
            except KeyboardInterrupt:
                print("\n\nInterrupted by user.")
                break
    
    def run_auto_scan(self):
        """Run a single scan and exit"""
        if self.connect():
            self.start_scan()
            self.disconnect()
    
    def run_interactive(self):
        """Run interactive mode"""
        if self.connect():
            try:
                self.interactive_mode()
            finally:
                self.disconnect()


def main():
    parser = argparse.ArgumentParser(description='Ultrasonic 360° Scan Test')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0',
                        help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('-b', '--baud', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('-a', '--auto', action='store_true',
                        help='Run single scan and exit (default: interactive mode)')
    
    args = parser.parse_args()
    
    test = UltrasonicTest(port=args.port, baud=args.baud)
    
    if args.auto:
        test.run_auto_scan()
    else:
        test.run_interactive()


if __name__ == '__main__':
    main()
