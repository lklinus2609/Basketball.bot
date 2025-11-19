"""
Main state machine for basketball robot
Optimized for 1-second shot cycle with predictive shooting
"""

import time
from enum import Enum, auto
from ballistics import ballistics
import math
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
    ROTATION_SPEED,
    SCAN_SAMPLES,
    SCAN_DELAY_MS,
    MIN_VALID_DISTANCE,
    MAX_VALID_DISTANCE,
    ARENA_WIDTH,
    ARENA_LENGTH,
    BASKET_POSITIONS,
    USE_TRIANGULATION,
    STAY_IN_PLACE,
    RETURN_TO_CENTER_AFTER_SHOT
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
        self.robot_position = None  # (x, y) position in arena
        self.robot_heading = 0.0    # Angle robot is facing (0° = facing +X direction)

        # Pre-calculated shot parameters for each basket
        self.shot_parameters = {
            'LEFT': {'distance': 0, 'rpm': 0, 'lazy_susan_angle': 0},
            'CENTER': {'distance': 0, 'rpm': 0, 'lazy_susan_angle': 0},
            'RIGHT': {'distance': 0, 'rpm': 0, 'lazy_susan_angle': 0}
        }

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
                print(f"[STATE] {self.state.name} -> {new_state.name} ({duration_ms}ms)")

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
                print("[STATE] Initialization complete - starting localization")
            self.transition_to(State.SEEK_AND_ORIENT)

    def _state_seek_and_orient(self):
        """
        SMART Localization Strategy:
        1. Scan 360° to get distances to all 4 walls
        2. Calculate exact (x,y) position using triangulation
        3. Rotate to face center hoop
        4. Pre-calculate distances, RPMs, and angles for ALL 3 hoops
        5. Store for instant lookup during shooting
        """
        time_in_state = self.time_in_state()

        # Phase 1: 360° Scanning (0-3.6 seconds)
        if time_in_state < SCAN_SAMPLES * SCAN_DELAY_MS:
            # Start scan
            if self.scan_index == 0 and time_in_state < 100:
                if DEBUG_MODE:
                    print("[LOCALIZE] Starting 360° arena scan for triangulation...")
                # Start rotating in place
                self.comm.set_drive(-ROTATION_SPEED, ROTATION_SPEED)
                self.scan_readings = []

            # Collect distance samples
            if time_in_state >= self.scan_index * SCAN_DELAY_MS:
                distance = self.comm.get_distance_inches()

                # Record valid distances with angle
                if MIN_VALID_DISTANCE < distance < MAX_VALID_DISTANCE:
                    angle = (self.scan_index / SCAN_SAMPLES) * 360.0
                    self.scan_readings.append((angle, distance))

                    if DEBUG_MODE and self.scan_index % 18 == 0:  # Print every 90°
                        print(f"  {angle:.0f}°: {distance:.1f}\"")

                self.scan_index += 1
            return

        # Phase 2: Calculate Position & Pre-compute All Shot Parameters (3.6-5 seconds)
        elif time_in_state < (SCAN_SAMPLES * SCAN_DELAY_MS + 1500):
            # Stop rotating
            self.comm.set_drive(0, 0)

            if not self.scan_readings:
                if DEBUG_MODE:
                    print("[LOCALIZE] WARNING: No valid scan data!")
                self.is_localized = True
                self.transition_to(State.HUNT_FOR_TARGET)
                return

            # Find 4 walls (N, S, E, W) by finding peaks in each quadrant
            walls = self._extract_four_walls(self.scan_readings)

            if DEBUG_MODE:
                print(f"[LOCALIZE] Detected walls:")
                for direction, (angle, dist) in walls.items():
                    print(f"  {direction}: {angle:.0f}° @ {dist:.1f}\"")

            # Calculate robot position from wall distances
            self.robot_position = self._calculate_position_from_walls(walls)

            if self.robot_position:
                x, y = self.robot_position
                if DEBUG_MODE:
                    print(f"[LOCALIZE] Calculated position: ({x:.1f}\", {y:.1f}\")")

                # Pre-calculate shot parameters for ALL 3 hoops
                self._precalculate_shot_parameters()

                if DEBUG_MODE:
                    print(f"[LOCALIZE] Pre-calculated shot parameters:")
                    for basket, params in self.shot_parameters.items():
                        print(f"  {basket}: {params['distance']:.1f}\" @ {params['rpm']} RPM, "
                              f"angle {params['lazy_susan_angle']:.1f}°")

            return

        # Phase 3: Rotate to Face Center Hoop (5-7 seconds)
        elif time_in_state < (SCAN_SAMPLES * SCAN_DELAY_MS + 3500):
            # Calculate angle to center hoop from current position
            if self.robot_position:
                center_angle = self.shot_parameters['CENTER']['lazy_susan_angle']

                # Use drive wheels for coarse alignment (big rotation)
                # Lazy susan will handle fine-tuning during shooting
                if abs(center_angle) > 15:  # Need robot rotation
                    if DEBUG_MODE and time_in_state < (SCAN_SAMPLES * SCAN_DELAY_MS + 1600):
                        print(f"[LOCALIZE] Rotating {center_angle:.0f}° to face center hoop...")

                    # Timed rotation (approximate)
                    rotation_duration = abs(center_angle) / 360.0 * 3.0  # 3 sec per 360°

                    if center_angle > 0:
                        self.comm.set_drive(-ROTATION_SPEED, ROTATION_SPEED)  # Clockwise
                    else:
                        self.comm.set_drive(ROTATION_SPEED, -ROTATION_SPEED)  # Counter-clockwise

                    time.sleep(rotation_duration)
                    self.comm.set_drive(0, 0)

                    # Update shot parameters (now robot is roughly centered)
                    # Lazy susan angles are now relative to this new heading
                    center_offset = center_angle
                    for basket in self.shot_parameters:
                        self.shot_parameters[basket]['lazy_susan_angle'] -= center_offset

                    if DEBUG_MODE:
                        print(f"[LOCALIZE] Aligned! Updated lazy susan angles:")
                        for basket, params in self.shot_parameters.items():
                            print(f"  {basket}: {params['lazy_susan_angle']:.1f}°")

            self.is_localized = True

            if DEBUG_MODE:
                print(f"[LOCALIZE] OK Localization complete! Ready to shoot.")

            self.transition_to(State.HUNT_FOR_TARGET)
            return

        # Timeout fallback
        else:
            self.comm.set_drive(0, 0)
            self.is_localized = True
            if DEBUG_MODE:
                print("[LOCALIZE] Timeout - proceeding anyway")
            self.transition_to(State.HUNT_FOR_TARGET)

    def _extract_four_walls(self, scan_readings):
        """Extract 4 wall distances from 360° scan"""
        # Divide scan into 4 quadrants and find max in each (furthest = wall)
        quadrants = {
            'EAST': [],   # 0-90° (backboard side with hoops)
            'NORTH': [],  # 90-180°
            'WEST': [],   # 180-270°
            'SOUTH': []   # 270-360°
        }

        for angle, dist in scan_readings:
            if 0 <= angle < 90:
                quadrants['EAST'].append((angle, dist))
            elif 90 <= angle < 180:
                quadrants['NORTH'].append((angle, dist))
            elif 180 <= angle < 270:
                quadrants['WEST'].append((angle, dist))
            else:
                quadrants['SOUTH'].append((angle, dist))

        walls = {}
        for direction, readings in quadrants.items():
            if readings:
                # Find max distance in quadrant (furthest = wall)
                walls[direction] = max(readings, key=lambda x: x[1])

        return walls

    def _calculate_position_from_walls(self, walls):
        """Calculate (x, y) position from wall distances"""
        # Arena coordinate system:
        # (0, 0) = back-left corner (from robot's side)
        # +X = toward hoops (backboard at x=72)
        # +Y = toward right (left edge at y=0, right edge at y=72)

        try:
            # Extract wall distances
            dist_east = walls.get('EAST', (0, 72))[1]   # Distance to backboard
            dist_west = walls.get('WEST', (0, 0))[1]    # Distance to back wall
            dist_north = walls.get('NORTH', (0, 72))[1] # Distance to left wall
            dist_south = walls.get('SOUTH', (0, 72))[1] # Distance to right wall

            # Calculate position
            x = ARENA_LENGTH - dist_east  # How far from back wall
            y = dist_north                # How far from left wall

            # Sanity check
            if 0 <= x <= ARENA_LENGTH and 0 <= y <= ARENA_WIDTH:
                return (x, y)
            else:
                return None
        except:
            return None

    def _precalculate_shot_parameters(self):
        """Pre-calculate distance, RPM, and lazy susan angle for all 3 hoops"""
        if not self.robot_position:
            return

        robot_x, robot_y = self.robot_position

        for basket_name, (basket_x, basket_y) in BASKET_POSITIONS.items():
            # Calculate Euclidean distance
            distance = math.sqrt((basket_x - robot_x)**2 + (basket_y - robot_y)**2)

            # Calculate angle to basket (relative to robot's forward direction)
            angle_to_basket = math.degrees(math.atan2(basket_y - robot_y, basket_x - robot_x))

            # Get required RPM from ballistics lookup table
            rpm = ballistics.get_required_rpm(distance)

            # Store pre-calculated parameters
            self.shot_parameters[basket_name] = {
                'distance': distance,
                'rpm': rpm,
                'lazy_susan_angle': angle_to_basket
            }

    def _state_hunt_for_target(self):
        """Wait for active beacon to appear"""
        # Keep lazy susan pointed at center (0°) for consistent ready position
        # This is the "home" position between shots
        # Flywheels running at idle speed

        # Ensure lazy susan is at center (in case it drifted)
        if RETURN_TO_CENTER_AFTER_SHOT and self.time_in_state() < 100:
            self.comm.rotate_lazy_susan(0.0)

        # Check for active beacon
        active_beacon = self.comm.get_active_beacon()

        if active_beacon is not None:
            self.target_beacon = active_beacon
            if DEBUG_MODE:
                print(f"[STATE] Beacon detected: {active_beacon}")
            self.transition_to(State.CALCULATE_SHOT)

    def _state_calculate_shot(self):
        """INSTANT shot parameter lookup - already pre-calculated!"""

        # Look up pre-calculated parameters for this basket
        if self.target_beacon in self.shot_parameters:
            params = self.shot_parameters[self.target_beacon]

            self.target_distance = params['distance']
            self.target_rpm = params['rpm']
            self.target_angle = params['lazy_susan_angle']

            if DEBUG_MODE:
                print(f"[STATE] Target: {self.target_beacon} (PRE-CALCULATED)")
                print(f"  Distance: {self.target_distance:.1f}\"")
                print(f"  RPM: {self.target_rpm}")
                print(f"  Lazy Susan: {self.target_angle:.1f}°")

        else:
            # Fallback if pre-calculation failed
            if DEBUG_MODE:
                print(f"[STATE] WARNING: No pre-calculated params for {self.target_beacon}, using fallback")

            self.target_distance = self.comm.get_distance_inches()
            self.target_rpm = ballistics.get_required_rpm(self.target_distance)
            self.target_angle = ballistics.get_lazy_susan_angle(self.target_beacon)

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
                    print("  WARNING: Exceeding 1-second target!")

            # Return to center position after shot for consistent ready state
            if RETURN_TO_CENTER_AFTER_SHOT:
                if DEBUG_MODE:
                    print(f"[STATE] Returning lazy susan to center (0°)...")
                self.comm.rotate_lazy_susan(0.0)

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
