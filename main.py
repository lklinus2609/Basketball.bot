#!/usr/bin/env python3
"""
Basketball Robot - Main Entry Point
ME 348E/392Q Advanced Mechatronics - Fall 2025

Usage:
    python3 main.py
"""

import time
import signal
import sys
from communication import AStarCommunication
from state_machine import BasketballRobotStateMachine
from config import (
    STATE_MACHINE_UPDATE_RATE,
    DEBUG_MODE,
    GAME_DURATION_SECONDS,
    ENABLE_GAME_TIMER
)


class BasketballRobot:
    """Main robot controller"""

    def __init__(self):
        print("\n" + "="*60)
        print("BASKETBALL ROBOT - BEVO MADNESS")
        print("="*60 + "\n")

        # Initialize communication
        print("[INIT] Connecting to A-Star...")
        self.comm = AStarCommunication()

        if not self.comm.connect():
            print("[ERROR] Failed to connect to A-Star!")
            print("  Check:")
            print("  - Serial port in config.py")
            print("  - A-Star is powered on")
            print("  - Serial connection (USB/UART)")
            sys.exit(1)

        print("[INIT] OK Connected to A-Star")

        # Initialize state machine
        print("[INIT] Initializing state machine...")
        self.state_machine = BasketballRobotStateMachine(self.comm)
        print("[INIT] OK State machine ready")

        # Setup signal handler for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)

        self.running = False
        self.update_interval = STATE_MACHINE_UPDATE_RATE / 1000.0  # Convert to seconds

        print("\n[INIT] Initialization complete!")
        print("="*60 + "\n")

    def _signal_handler(self, sig, frame):
        """Handle Ctrl+C for clean shutdown"""
        print("\n[MAIN] Shutdown signal received...")
        self.stop()

    def run(self):
        """Main run loop"""
        self.running = True

        # Wait for user to start the game
        print("\n" + "="*60)
        print("READY TO START")
        print("="*60)
        print("\nPlace robot in starting zone, then press '1' to begin...")
        print("(or type 'q' to quit)\n")

        try:
            while True:
                user_input = input(">>> ").strip().lower()
                if user_input == 'q':
                    print("[MAIN] Cancelled by user")
                    self.stop()
                    return
                elif user_input == '1':
                    break
                else:
                    print("Please press '1' to start or 'q' to quit.")
        except KeyboardInterrupt:
            print("\n[MAIN] Cancelled by user")
            self.stop()
            return

        print("\n" + "="*60)
        print("STARTING AUTONOMOUS SEQUENCE!")
        print("="*60 + "\n")

        # Now start the state machine
        self.state_machine.start()

        print("[MAIN] Starting main loop...")
        print(f"[MAIN] Update rate: {1/self.update_interval:.0f} Hz")

        if ENABLE_GAME_TIMER:
            print(f"[MAIN] Game timer: {GAME_DURATION_SECONDS} seconds\n")

        last_update = time.time()

        try:
            while self.running:
                current_time = time.time()

                # Maintain consistent update rate
                if current_time - last_update >= self.update_interval:
                    # Update state machine
                    self.state_machine.update()

                    # Check game timer
                    if ENABLE_GAME_TIMER:
                        game_time = self.state_machine.get_game_time()
                        if game_time >= GAME_DURATION_SECONDS:
                            print(f"\n[MAIN] Game time expired ({GAME_DURATION_SECONDS}s)")
                            self.state_machine.emergency_stop()
                            self.running = False

                    last_update = current_time

                else:
                    # Small sleep to prevent CPU spinning
                    time.sleep(0.001)

        except Exception as e:
            print(f"\n[ERROR] Exception in main loop: {e}")
            import traceback
            traceback.print_exc()

        finally:
            self.stop()

    def stop(self):
        """Clean shutdown"""
        print("\n[MAIN] Shutting down...")

        # Stop all motion
        if self.comm.connected:
            print("[MAIN] Stopping motors...")
            self.comm.set_flywheel_rpm(0)
            self.comm.set_drive(0, 0)
            time.sleep(0.1)

            print("[MAIN] Disconnecting from A-Star...")
            self.comm.disconnect()

        self.running = False
        print("[MAIN] Shutdown complete.\n")


def main():
    """Main entry point"""
    robot = BasketballRobot()

    try:
        robot.run()
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] Fatal error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.stop()

    print("Goodbye!")
    sys.exit(0)


if __name__ == "__main__":
    main()
