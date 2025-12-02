#!/usr/bin/env python3
"""
Stepper Motor Diagnostic Script
Continuously triggers the loader stepper to test torque and speed.
"""

import time
import signal
import sys
from communication import AStarCommunication

# Configuration
STEP_INTERVAL = 0.5  # Seconds between shots (adjust as needed)

def signal_handler(sig, frame):
    print("\nStopping...")
    sys.exit(0)

def main():
    print("========================================")
    print("STEPPER MOTOR DIAGNOSTIC TEST")
    print("========================================")
    print(f"Interval: {STEP_INTERVAL}s")
    print("Press Ctrl+C to stop")
    print("----------------------------------------")

    # Initialize Communication
    comm = AStarCommunication()
    if not comm.connect():
        print("ERROR: Could not connect to A-Star.")
        return

    # Register signal handler for clean exit
    signal.signal(signal.SIGINT, signal_handler)

    count = 0
    try:
        while True:
            count += 1
            print(f"[{count}] Firing stepper...")
            
            # Trigger Shot
            comm.shoot()
            
            # Wait for move to complete + buffer
            time.sleep(STEP_INTERVAL)
            
            # Read any debug output from Arduino
            comm.update()

    except KeyboardInterrupt:
        pass
    finally:
        comm.disconnect()
        print("\nTest Complete.")

if __name__ == "__main__":
    main()
