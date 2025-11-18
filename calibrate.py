#!/usr/bin/env python3
"""
Interactive Calibration Tool for Basketball Robot
Use this to manually test and calibrate your robot
"""

import time
import sys
from communication import AStarCommunication
from config import FLYWHEEL_IDLE_RPM

def print_menu():
    """Print interactive menu"""
    print("\n" + "="*60)
    print("BASKETBALL ROBOT - CALIBRATION TOOL")
    print("="*60)
    print("\nCommands:")
    print("  1  - Set flywheel RPM")
    print("  2  - Rotate lazy susan to angle")
    print("  3  - Fire loading mechanism (SHOOT)")
    print("  4  - Read all sensors")
    print("  5  - Monitor telemetry (live)")
    print("  6  - Test shot sequence")
    print("  7  - Measure encoder PPR")
    print("  0  - Stop all motors")
    print("  q  - Quit")
    print("="*60)

def set_flywheel_rpm(comm):
    """Manual flywheel RPM control"""
    print("\n--- Flywheel RPM Control ---")
    try:
        rpm = int(input("Enter target RPM (0-10000): "))
        if 0 <= rpm <= 10000:
            comm.set_flywheel_rpm(rpm)
            print(f"✓ Set flywheel RPM to {rpm}")

            # Wait and monitor
            print("Monitoring for 3 seconds...")
            for i in range(15):  # 3 seconds at 50Hz
                comm.update()
                time.sleep(0.2)
                if i % 5 == 4:  # Print every 1 second
                    rpm_left = comm.telemetry['flywheel_rpm_left']
                    rpm_right = comm.telemetry['flywheel_rpm_right']
                    print(f"  Current: L={rpm_left} RPM, R={rpm_right} RPM")
        else:
            print("✗ Invalid RPM (must be 0-10000)")
    except ValueError:
        print("✗ Invalid input")

def rotate_lazy_susan(comm):
    """Manual lazy susan control"""
    print("\n--- Lazy Susan Control ---")
    try:
        angle = float(input("Enter target angle in degrees (-180 to +180): "))
        if -180 <= angle <= 180:
            comm.rotate_lazy_susan(angle)
            print(f"✓ Rotating lazy susan to {angle}°")

            # Wait and monitor
            print("Monitoring for 3 seconds...")
            for i in range(15):
                comm.update()
                time.sleep(0.2)
                if i % 5 == 4:
                    current_angle = comm.telemetry['lazy_susan_angle']
                    print(f"  Current angle: {current_angle:.1f}°")
        else:
            print("✗ Invalid angle")
    except ValueError:
        print("✗ Invalid input")

def shoot(comm):
    """Fire loading mechanism"""
    print("\n--- Fire Loading Mechanism ---")
    confirm = input("Fire 90° shot? (y/n): ")
    if confirm.lower() == 'y':
        comm.shoot()
        print("✓ SHOOT command sent!")
        print("Monitoring for 2 seconds...")
        for i in range(10):
            comm.update()
            time.sleep(0.2)
            if i % 5 == 4:
                loader_ready = comm.telemetry['loader_ready']
                print(f"  Loader status: {'READY' if loader_ready else 'BUSY'}")

def read_sensors(comm):
    """Read all sensors once"""
    print("\n--- Sensor Readings ---")
    comm.update()
    time.sleep(0.1)
    comm.update()

    print(f"IR Sensors (active=0):")
    print(f"  Left:   {comm.telemetry['ir_left']}")
    print(f"  Center: {comm.telemetry['ir_center']}")
    print(f"  Right:  {comm.telemetry['ir_right']}")

    active = comm.get_active_beacon()
    print(f"  Active beacon: {active if active else 'None'}")

    print(f"\nUltrasonic:")
    print(f"  Distance: {comm.telemetry['distance_cm']} cm ({comm.get_distance_inches():.1f}\")")

    print(f"\nFlywheels:")
    print(f"  Left RPM:  {comm.telemetry['flywheel_rpm_left']}")
    print(f"  Right RPM: {comm.telemetry['flywheel_rpm_right']}")

    print(f"\nPosition:")
    print(f"  Lazy susan: {comm.telemetry['lazy_susan_angle']:.1f}°")
    print(f"  Loader: {'READY' if comm.telemetry['loader_ready'] else 'BUSY'}")

def monitor_telemetry(comm):
    """Live telemetry monitoring"""
    print("\n--- Live Telemetry Monitor ---")
    print("Press Ctrl+C to stop\n")

    try:
        while True:
            comm.update()

            # Clear line and print telemetry
            sys.stdout.write('\r' + ' '*80 + '\r')  # Clear line
            sys.stdout.write(
                f"RPM: L={comm.telemetry['flywheel_rpm_left']:4d} R={comm.telemetry['flywheel_rpm_right']:4d} | "
                f"Angle: {comm.telemetry['lazy_susan_angle']:6.1f}° | "
                f"Dist: {comm.telemetry['distance_cm']:3d}cm | "
                f"IR: {comm.telemetry['ir_left']}{comm.telemetry['ir_center']}{comm.telemetry['ir_right']} | "
                f"Loader: {'RDY' if comm.telemetry['loader_ready'] else 'BSY'}"
            )
            sys.stdout.flush()

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nStopped monitoring.")

def test_shot_sequence(comm):
    """Test a full shot sequence"""
    print("\n--- Test Shot Sequence ---")
    print("This will execute a complete shot cycle:")
    print("  1. Set flywheel RPM")
    print("  2. Rotate lazy susan")
    print("  3. Wait for stabilization")
    print("  4. Fire shot")

    confirm = input("\nProceed? (y/n): ")
    if confirm.lower() != 'y':
        return

    # Get parameters
    try:
        rpm = int(input("Target RPM: "))
        angle = float(input("Target angle: "))
    except ValueError:
        print("✗ Invalid input")
        return

    print("\n--- Executing Shot Sequence ---")

    # Step 1: Set RPM and angle
    print("1. Setting flywheel RPM and lazy susan angle...")
    comm.set_flywheel_rpm(rpm)
    comm.rotate_lazy_susan(angle)

    # Step 2: Wait for stabilization
    print("2. Waiting for stabilization (3 seconds)...")
    start_time = time.time()
    stable_count = 0

    while time.time() - start_time < 3.0:
        comm.update()

        rpm_current = (comm.telemetry['flywheel_rpm_left'] +
                       comm.telemetry['flywheel_rpm_right']) / 2
        angle_current = comm.telemetry['lazy_susan_angle']

        rpm_error = abs(rpm_current - rpm) / rpm * 100 if rpm > 0 else 0
        angle_error = abs(angle_current - angle)

        if rpm_error < 3.0 and angle_error < 3.0:
            stable_count += 1

        if stable_count > 5:  # Stable for 5 readings
            print(f"   ✓ Stable! RPM error: {rpm_error:.1f}%, Angle error: {angle_error:.1f}°")
            break

        time.sleep(0.1)

    # Step 3: Shoot
    print("3. Firing shot...")
    comm.shoot()
    time.sleep(0.5)

    # Step 4: Monitor completion
    print("4. Monitoring shot completion...")
    for i in range(10):
        comm.update()
        time.sleep(0.2)
        if comm.telemetry['loader_ready']:
            print("   ✓ Shot complete! Loader ready.")
            break

    print("\n✓ Shot sequence complete!")

def measure_encoder_ppr(comm):
    """Help measure encoder PPR"""
    print("\n--- Encoder PPR Measurement ---")
    print("This tool helps you measure encoder pulses per revolution (PPR)")
    print("\nProcedure:")
    print("  1. Manually rotate the flywheel exactly 10 revolutions")
    print("  2. Watch the encoder count in the A-Star debug output")
    print("  3. PPR = (final count - initial count) / 10")
    print("\nAlternatively:")
    print("  1. Set flywheels to low RPM (e.g., 1000)")
    print("  2. Count rotations for 10 seconds")
    print("  3. Calculate: RPM = rotations * 6")
    print("  4. Compare to reported RPM from A-Star")
    print("  5. Adjust ENCODER_PPR until they match")

    input("\nPress Enter to continue...")

def stop_all(comm):
    """Emergency stop"""
    print("\n--- STOPPING ALL MOTORS ---")
    comm.set_flywheel_rpm(0)
    comm.rotate_lazy_susan(0)
    comm.set_drive(0, 0)
    print("✓ All motors stopped")

def main():
    """Main calibration tool"""
    print("\n" + "="*60)
    print("Connecting to A-Star...")
    print("="*60)

    comm = AStarCommunication()

    if not comm.connect():
        print("\n✗ ERROR: Failed to connect to A-Star!")
        print("Check:")
        print("  - Serial port in config.py")
        print("  - A-Star is powered on")
        print("  - Correct USB/UART connection")
        sys.exit(1)

    print("✓ Connected to A-Star!\n")

    # Main loop
    try:
        while True:
            print_menu()
            choice = input("\nEnter command: ").strip()

            # Keep communication alive
            comm.update()

            if choice == '1':
                set_flywheel_rpm(comm)
            elif choice == '2':
                rotate_lazy_susan(comm)
            elif choice == '3':
                shoot(comm)
            elif choice == '4':
                read_sensors(comm)
            elif choice == '5':
                monitor_telemetry(comm)
            elif choice == '6':
                test_shot_sequence(comm)
            elif choice == '7':
                measure_encoder_ppr(comm)
            elif choice == '0':
                stop_all(comm)
            elif choice.lower() == 'q':
                print("\nQuitting...")
                stop_all(comm)
                break
            else:
                print("✗ Invalid command")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    finally:
        print("\nCleaning up...")
        stop_all(comm)
        comm.disconnect()
        print("Goodbye!\n")

if __name__ == "__main__":
    main()
