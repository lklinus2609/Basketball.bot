"""
Main state machine for basketball robot
Optimized for 1-second shot cycle with predictive shooting
"""

import time
from enum import Enum, auto
from ballistics import ballistics
import math
import RPi.GPIO as GPIO
from config import (
    CALCULATE_SHOT_TIMEOUT,
    ALIGN_SPINUP_TIMEOUT,
    STABILIZATION_TIME,
    SHOOT_DURATION,
    FLYWHEEL_IDLE_RPM,
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

# IR Sensor Pin (BCM)
IR_SENSOR_PIN = 17

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

        # GPIO Setup for IR Sensor
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(IR_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # IR Filter State (Simple hysteresis)
        self.ir_samples = []
        self.ir_filtered_state = 1 # 1 = Not Detected

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
            self.comm.set_flywheel_rpm(FLYWHEEL_IDLE_RPM) # No-op now, but keeps protocol happy

        elif self.state == State.HUNT_FOR_TARGET:
            # Start panning
            self.comm.start_panning(75) # Speed 75

        elif self.state == State.CALCULATE_SHOT:
            # Mark timing for performance tracking
            self.beacon_detected_time = time.time()

        # Note: EXECUTE_SHOT doesn't need state entry action
        # Shooting is triggered in the state update loop with loader ready check

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
        Simplified Localization:
        Just transition to hunting for now.
        Full localization can be re-enabled if needed.
        """
        # For now, skip complex localization and just start hunting
        self.is_localized = True
        self.transition_to(State.HUNT_FOR_TARGET)

    def _state_hunt_for_target(self):
        """
        PANNING MODE:
        - Read IR sensor on Pi
        - Send IR state to Arduino (Arduino handles stopping)
        - Wait for Arduino to report stopped/detected
        """
        # Read IR Sensor
        ir_val = GPIO.input(IR_SENSOR_PIN)
        
        # Update filter (simple window)
        self.ir_samples.append(ir_val)
        if len(self.ir_samples) > 20:
            self.ir_samples.pop(0)
            
        zeros = self.ir_samples.count(0)
        
        # Hysteresis Logic (Same as IRscan.py)
        if self.ir_filtered_state == 1: # Not Detected
            if zeros >= 4: # 20%
                self.ir_filtered_state = 0 # Detected
        else: # Detected
            if zeros < 2: # 10%
                self.ir_filtered_state = 1 # Not Detected
                
        # Send state to Arduino
        self.comm.set_ir_state(self.ir_filtered_state == 0)
        
        # Check if we have found a target (Filtered state is DETECTED)
        if self.ir_filtered_state == 0:
            # We are detected. Wait a moment to ensure stability?
            # Actually, we should transition to CALCULATE_SHOT
            # But we need to make sure the robot has actually STOPPED.
            # The Arduino stops motors immediately upon receiving IR state 0.
            
            if DEBUG_MODE and self.time_in_state() % 500 < 20: # Log occasionally
                print(f"[STATE] IR DETECTED! Handing off to shot calc...")
                
            self.target_beacon = "CENTER" # Assume center for now, or use multi-sensor logic if we had 3 sensors
            self.transition_to(State.CALCULATE_SHOT)

    def _state_calculate_shot(self):
        """Calculate shot parameters"""
        # Just get distance and set RPM (though RPM is external now)
        self.target_distance = self.comm.get_distance_inches()
        
        # We don't control RPM anymore, but we can calculate it for logging
        self.target_rpm = ballistics.get_required_rpm(self.target_distance)

        if DEBUG_MODE:
            print(f"[STATE] Target Distance: {self.target_distance:.1f}\"")
            
        # Immediately transition
        self.transition_to(State.ALIGN_AND_SPINUP)

    def _state_align_and_spinup(self):
        """Wait for stability before shooting"""
        # Since we are already stopped (IR detected), we just need to wait a moment
        # to ensure the robot is stable and not wobbling.
        
        if self.time_in_state() > STABILIZATION_TIME:
             self.transition_to(State.EXECUTE_SHOT)

    def _state_execute_shot(self):
        """Execute the shot and wait for completion"""
        # Trigger shot on first update cycle
        if self.time_in_state() < 50:
            if self.comm.is_loader_ready():
                self.comm.shoot()
                if DEBUG_MODE:
                    print(f"[STATE] FIRING shot #{self.shots_made + 1}!")
            else:
                if DEBUG_MODE:
                    print(f"[STATE] WARNING: Loader not ready when entering EXECUTE_SHOT!")

        # Wait for loader to complete
        is_ready = self.comm.is_loader_ready()

        # Exit when loader completes OR timeout after 1 second
        if (is_ready and self.time_in_state() > 100) or self.time_in_state() > 1000:
            # Shot complete!
            self.balls_remaining -= 1
            self.shots_made += 1

            # Log shot timing
            if LOG_SHOT_TIMING and self.beacon_detected_time > 0:
                shot_duration = int((time.time() - self.beacon_detected_time) * 1000)
                self.shot_times.append(shot_duration)
                print(f"[TIMING] Shot #{self.shots_made}: {shot_duration}ms")

            # Check if more balls remaining
            if self.balls_remaining > 0:
                if DEBUG_MODE:
                    print(f"[STATE] Shot complete! {self.balls_remaining} balls remaining")
                self.transition_to(State.HUNT_FOR_TARGET)
            else:
                print(f"[STATE] All balls shot! Total shots: {self.shots_made}")
                self.transition_to(State.END_GAME)

    def _state_end_game(self):
        """Game over - stop"""
        if self.time_in_state() < 100:  # Only do this once
            self.comm.stop_panning()       # Ensure panning is stopped
            self.comm.set_drive(0, 0)      # Stop driving

            # Print final stats
            game_duration = time.time() - self.game_start_time
            print("\n" + "="*60)
            print("GAME COMPLETE!")
            print("="*60)
            print(f"Shots made: {self.shots_made}")
            print(f"Balls remaining: {self.balls_remaining}")
            print(f"Game duration: {game_duration:.1f}s")
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
        self.comm.stop_panning()
        self.comm.set_drive(0, 0)
        self.transition_to(State.END_GAME)
