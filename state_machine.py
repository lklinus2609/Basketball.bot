"""
Main state machine for basketball robot
Optimized for 1-second shot cycle with predictive shooting
"""

import time
from enum import Enum, auto
from ballistics import ballistics
from config import (
    CALCULATE_SHOT_TIMEOUT,
    ALIGN_SPINUP_TIMEOUT,
    STABILIZATION_TIME,
    SHOOT_DURATION,
    FLYWHEEL_IDLE_RPM,
    IDLE_LAZY_SUSAN_ANGLE,
    LAZY_SUSAN_ANGLE_TOLERANCE,
    FLYWHEEL_RPM_TOLERANCE,
    BALL_COUNT,
    DEBUG_MODE,
    LOG_SHOT_TIMING,
    PREDICTIVE_SHOOT_ADVANCE,
    OPTIMAL_SHOOTING_DISTANCE,
    DISTANCE_TOLERANCE,
    ROTATION_SPEED,
    DRIVE_SPEED_SLOW,
    DRIVE_SPEED_FAST,
    SCAN_SAMPLES,
    SCAN_DELAY_MS,
    MIN_VALID_DISTANCE,
    MAX_VALID_DISTANCE
)


class State(Enum):
    """Robot states"""
    INIT = auto()
    SEEK_AND_ORIENT = auto()
    HUNT_FOR_TARGET = auto()
    CALCULATE_SHOT = auto()
    ALIGN_AND_SPINUP = auto()
    EXECUTE_SHOT = auto()
    END_GAME = auto()


class BasketballRobotStateMachine:
    """Main state machine controller"""

    def __init__(self, comm):
        """
        Initialize state machine

        Args:
            comm: AStarCommunication instance
        """
        self.comm = comm
        self.state = State.INIT
        self.previous_state = None
        self.state_entry_time = 0
        self.game_start_time = 0

        # Game state
        self.balls_remaining = BALL_COUNT
        self.shots_made = 0

        # Current target info
        self.target_beacon = None
        self.target_distance = 0
        self.target_rpm = 0
        self.target_angle = 0

        # Timing tracking
        self.beacon_detected_time = 0
        self.shot_times = []

        # Stability tracking for predictive shooting
        self.stable_start_time = None

        # Localization state
        self.is_localized = False
        self.scan_readings = []
        self.scan_index = 0
        self.backboard_angle = None

        if DEBUG_MODE:
            print("[STATE] State machine initialized")

    def start(self):
        """Start the state machine"""
        self.transition_to(State.INIT)
        self.game_start_time = time.time()
        if DEBUG_MODE:
            print("[STATE] Game started!")

    def transition_to(self, new_state):
        """
        Transition to new state with timing tracking

        Args:
            new_state: State to transition to
        """
        # Log state duration
        if self.state_entry_time > 0:
            duration_ms = int((time.time() - self.state_entry_time) * 1000)
            if DEBUG_MODE:
                print(f"[STATE] {self.state.name} → {new_state.name} ({duration_ms}ms)")

        self.previous_state = self.state
        self.state = new_state
        self.state_entry_time = time.time()

        # Reset state-specific variables
        self.stable_start_time = None

        # State entry actions
        self._on_state_entry()

    def _on_state_entry(self):
        """Actions to perform when entering a state"""
        if self.state == State.INIT:
            # Initialize robot systems
            self.comm.set_flywheel_rpm(FLYWHEEL_IDLE_RPM)
            self.comm.rotate_lazy_susan(IDLE_LAZY_SUSAN_ANGLE)

        elif self.state == State.HUNT_FOR_TARGET:
            # Return to idle position
            self.comm.rotate_lazy_susan(IDLE_LAZY_SUSAN_ANGLE)
            # Keep flywheels spinning at idle or last speed

        elif self.state == State.CALCULATE_SHOT:
            # Mark timing for performance tracking
            self.beacon_detected_time = time.time()

        elif self.state == State.EXECUTE_SHOT:
            # Trigger the shot
            self.comm.shoot()

    def time_in_state(self):
        """Get time spent in current state (milliseconds)"""
        return int((time.time() - self.state_entry_time) * 1000)

    def update(self):
        """
        Main state machine update - call every loop iteration
        Non-blocking, returns immediately
        """
        # Update communication (read telemetry)
        self.comm.update()

        # State machine logic
        if self.state == State.INIT:
            self._state_init()
        elif self.state == State.SEEK_AND_ORIENT:
            self._state_seek_and_orient()
        elif self.state == State.HUNT_FOR_TARGET:
            self._state_hunt_for_target()
        elif self.state == State.CALCULATE_SHOT:
            self._state_calculate_shot()
        elif self.state == State.ALIGN_AND_SPINUP:
            self._state_align_and_spinup()
        elif self.state == State.EXECUTE_SHOT:
            self._state_execute_shot()
        elif self.state == State.END_GAME:
            self._state_end_game()

    # ========================================================================
    # STATE IMPLEMENTATIONS
    # ========================================================================

    def _state_init(self):
        """Initialize and wait for systems to be ready"""
        # Wait for initial commands to be processed
        if self.time_in_state() > 500:  # 500ms startup delay
            if DEBUG_MODE:
                print("[STATE] Initialization complete")
            self.transition_to(State.SEEK_AND_ORIENT)

    def _state_seek_and_orient(self):
        """
        Initial localization and positioning

        Strategy:
        1. Scan 360° with ultrasonic to find backboard (shortest distance)
        2. Rotate to face backboard
        3. Drive to optimal shooting distance
        4. Verify IR beacons are visible
        """
        time_in_state = self.time_in_state()

        # Phase 1: Scanning (0-4 seconds)
        if time_in_state < SCAN_SAMPLES * SCAN_DELAY_MS:
            # Perform 360° scan
            if self.scan_index == 0 and time_in_state < 100:
                if DEBUG_MODE:
                    print("[LOCALIZE] Starting 360° scan...")
                # Start rotating
                self.comm.set_drive(-ROTATION_SPEED, ROTATION_SPEED)  # Rotate in place
                self.scan_readings = []

            # Collect scan samples
            if time_in_state >= self.scan_index * SCAN_DELAY_MS:
                distance = self.comm.get_distance_inches()

                # Only record valid distances
                if MIN_VALID_DISTANCE < distance < MAX_VALID_DISTANCE:
                    angle = (self.scan_index / SCAN_SAMPLES) * 360.0
                    self.scan_readings.append((angle, distance))

                    if DEBUG_MODE and self.scan_index % 6 == 0:  # Print every 60°
                        print(f"  Scan: {angle:.0f}° = {distance:.1f}\"")

                self.scan_index += 1

            return

        # Phase 2: Analyze scan and orient (4-5 seconds)
        elif time_in_state < (SCAN_SAMPLES * SCAN_DELAY_MS + 1000):
            # Stop rotating
            self.comm.set_drive(0, 0)

            if not self.scan_readings:
                # No valid readings - try simple strategy
                if DEBUG_MODE:
                    print("[LOCALIZE] No valid scan data, using fallback")
                self.is_localized = True  # Give up and proceed
                self.transition_to(State.HUNT_FOR_TARGET)
                return

            # Find minimum distance (backboard)
            self.backboard_angle, min_distance = min(self.scan_readings, key=lambda x: x[1])

            if DEBUG_MODE:
                print(f"[LOCALIZE] Backboard detected at angle {self.backboard_angle:.0f}°, "
                      f"distance {min_distance:.1f}\"")

            # Calculate rotation needed (use drive wheels to rotate)
            # Rotate to face backboard (minimum distance direction)
            rotation_time = (self.backboard_angle / 360.0) * 3000  # ~3s for full rotation

            if self.backboard_angle > 10:  # Need to rotate
                if DEBUG_MODE:
                    print(f"[LOCALIZE] Rotating {self.backboard_angle:.0f}° to face backboard...")
                self.comm.set_drive(-ROTATION_SPEED, ROTATION_SPEED)
                time.sleep(rotation_time / 1000.0)  # Simple timed rotation
                self.comm.set_drive(0, 0)

            return

        # Phase 3: Position at optimal distance (5-8 seconds)
        elif time_in_state < (SCAN_SAMPLES * SCAN_DELAY_MS + 4000):
            current_distance = self.comm.get_distance_inches()
            distance_error = current_distance - OPTIMAL_SHOOTING_DISTANCE

            if abs(distance_error) < DISTANCE_TOLERANCE:
                # At optimal position!
                self.comm.set_drive(0, 0)
                self.is_localized = True

                if DEBUG_MODE:
                    print(f"[LOCALIZE] Positioned at {current_distance:.1f}\" from backboard ✓")

                # Check if we can see beacons
                active_beacon = self.comm.get_active_beacon()
                if active_beacon or time_in_state > (SCAN_SAMPLES * SCAN_DELAY_MS + 3000):
                    # Either see beacon or timeout - proceed
                    if DEBUG_MODE:
                        print(f"[LOCALIZE] Localization complete! "
                              f"Beacon visible: {active_beacon if active_beacon else 'waiting...'}")
                    self.transition_to(State.HUNT_FOR_TARGET)

            elif distance_error > 0:
                # Too far - drive forward
                if DEBUG_MODE and time_in_state % 500 < 50:
                    print(f"[LOCALIZE] Moving forward {distance_error:.1f}\" to target...")
                self.comm.set_drive(DRIVE_SPEED_SLOW, DRIVE_SPEED_SLOW)
            else:
                # Too close - drive backward
                if DEBUG_MODE and time_in_state % 500 < 50:
                    print(f"[LOCALIZE] Moving back {-distance_error:.1f}\" to target...")
                self.comm.set_drive(-DRIVE_SPEED_SLOW, -DRIVE_SPEED_SLOW)

        # Timeout - just proceed anyway
        else:
            self.comm.set_drive(0, 0)
            self.is_localized = True

            if DEBUG_MODE:
                print("[LOCALIZE] Timeout - proceeding to hunting")
            self.transition_to(State.HUNT_FOR_TARGET)

    def _state_hunt_for_target(self):
        """Wait for active beacon to appear"""
        # Keep lazy susan pointed at center (minimize rotation needed)
        # Flywheels running at idle speed

        # Check for active beacon
        active_beacon = self.comm.get_active_beacon()

        if active_beacon is not None:
            self.target_beacon = active_beacon
            if DEBUG_MODE:
                print(f"[STATE] Beacon detected: {active_beacon}")
            self.transition_to(State.CALCULATE_SHOT)

    def _state_calculate_shot(self):
        """Fast calculation of shot parameters"""
        # Measure distance
        self.target_distance = self.comm.get_distance_inches()

        # Calculate required RPM
        self.target_rpm = ballistics.get_required_rpm(self.target_distance)

        # Calculate lazy susan angle
        self.target_angle = ballistics.get_lazy_susan_angle(self.target_beacon)

        if DEBUG_MODE:
            print(f"[STATE] Target: {self.target_beacon}, "
                  f"Distance: {self.target_distance:.1f}\", "
                  f"RPM: {self.target_rpm}, "
                  f"Angle: {self.target_angle}°")

        # Send commands (non-blocking, executed in parallel)
        self.comm.set_flywheel_rpm(self.target_rpm)
        self.comm.rotate_lazy_susan(self.target_angle)

        # Immediately transition (don't wait)
        self.transition_to(State.ALIGN_AND_SPINUP)

    def _state_align_and_spinup(self):
        """Wait for lazy susan and flywheels to reach target"""
        # Check current status
        is_aligned = self.comm.is_aligned(
            self.target_angle,
            LAZY_SUSAN_ANGLE_TOLERANCE
        )

        is_rpm_ready = self.comm.is_flywheel_ready(
            self.target_rpm,
            FLYWHEEL_RPM_TOLERANCE
        )

        # PREDICTIVE SHOOTING: Check if conditions are "good enough"
        if is_aligned and is_rpm_ready:
            # Start stability timer
            if self.stable_start_time is None:
                self.stable_start_time = time.time()

            # Check if stable for required duration
            stable_duration = (time.time() - self.stable_start_time) * 1000

            # Shoot EARLY by PREDICTIVE_SHOOT_ADVANCE ms
            if stable_duration >= (STABILIZATION_TIME - PREDICTIVE_SHOOT_ADVANCE):
                if DEBUG_MODE:
                    print(f"[STATE] Ready to shoot (stable for {stable_duration:.0f}ms)")
                self.transition_to(State.EXECUTE_SHOT)
        else:
            # Reset stability timer if conditions not met
            self.stable_start_time = None

        # Safety timeout: shoot anyway if taking too long
        if self.time_in_state() > ALIGN_SPINUP_TIMEOUT:
            current_angle = self.comm.telemetry['lazy_susan_angle']
            current_rpm = (self.comm.telemetry['flywheel_rpm_left'] +
                           self.comm.telemetry['flywheel_rpm_right']) / 2

            angle_error = abs(current_angle - self.target_angle)
            rpm_error = abs(current_rpm - self.target_rpm)

            print(f"[STATE] WARNING: Timeout shoot! "
                  f"Angle error: {angle_error:.1f}°, "
                  f"RPM error: {rpm_error:.0f}")

            self.transition_to(State.EXECUTE_SHOT)

    def _state_execute_shot(self):
        """Execute the shot and wait for completion"""
        # Wait for loading mechanism to complete
        is_ready = self.comm.is_loader_ready()

        if is_ready or self.time_in_state() > SHOOT_DURATION:
            # Shot complete!
            self.balls_remaining -= 1
            self.shots_made += 1

            # Log shot timing
            if LOG_SHOT_TIMING and self.beacon_detected_time > 0:
                shot_duration = int((time.time() - self.beacon_detected_time) * 1000)
                self.shot_times.append(shot_duration)

                avg_time = sum(self.shot_times) / len(self.shot_times)
                print(f"[TIMING] Shot #{self.shots_made}: {shot_duration}ms | "
                      f"Avg: {avg_time:.0f}ms | Target: 1000ms")

                if shot_duration > 1200:
                    print("  ⚠️  WARNING: Exceeding 1-second target!")

            # Check if more balls remaining
            if self.balls_remaining > 0:
                if DEBUG_MODE:
                    print(f"[STATE] Shot complete! {self.balls_remaining} balls remaining")
                self.transition_to(State.HUNT_FOR_TARGET)
            else:
                print(f"[STATE] All balls shot! Total shots: {self.shots_made}")
                self.transition_to(State.END_GAME)

    def _state_end_game(self):
        """Game over - spin down and stop"""
        if self.time_in_state() < 100:  # Only do this once
            self.comm.set_flywheel_rpm(0)  # Spin down
            self.comm.set_drive(0, 0)      # Stop driving

            # Print final stats
            game_duration = time.time() - self.game_start_time
            print("\n" + "="*60)
            print("GAME COMPLETE!")
            print("="*60)
            print(f"Shots made: {self.shots_made}")
            print(f"Balls remaining: {self.balls_remaining}")
            print(f"Game duration: {game_duration:.1f}s")

            if self.shot_times:
                avg_shot_time = sum(self.shot_times) / len(self.shot_times)
                min_shot_time = min(self.shot_times)
                max_shot_time = max(self.shot_times)

                print(f"\nShot timing stats:")
                print(f"  Average: {avg_shot_time:.0f}ms")
                print(f"  Fastest: {min_shot_time}ms")
                print(f"  Slowest: {max_shot_time}ms")

            print("="*60 + "\n")

    # ========================================================================
    # UTILITY METHODS
    # ========================================================================

    def get_game_time(self):
        """Get elapsed game time in seconds"""
        return time.time() - self.game_start_time

    def get_state_name(self):
        """Get current state name"""
        return self.state.name

    def emergency_stop(self):
        """Emergency stop - halt all motion"""
        print("[STATE] EMERGENCY STOP!")
        self.comm.set_flywheel_rpm(0)
        self.comm.set_drive(0, 0)
        self.transition_to(State.END_GAME)
