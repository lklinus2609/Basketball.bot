"""
Configuration file for Basketball Robot
All tunable parameters in one place for easy adjustment
"""

# ============================================================================
# COMMUNICATION SETTINGS
# ============================================================================
SERIAL_PORT = '/dev/ttyACM0'  # USB serial port for A-Star connection
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.01  # Non-blocking with minimal timeout

# ============================================================================
# TIMING PARAMETERS (all in milliseconds)
# ============================================================================
STATE_MACHINE_UPDATE_RATE = 20  # 50 Hz main loop
TELEMETRY_UPDATE_RATE = 20      # 50 Hz from A-Star

# State timeouts for 1-second shot cycle
CALCULATE_SHOT_TIMEOUT = 50     # Quick calculation phase
ALIGN_SPINUP_TIMEOUT = 600      # Max wait for alignment + flywheel
STABILIZATION_TIME = 500        # How long to hold stable before shooting
SHOOT_DURATION = 350            # Loading mechanism rotation time

# Predictive shooting: shoot this many ms BEFORE perfect alignment
PREDICTIVE_SHOOT_ADVANCE = 50   # Shoot 50ms early (ball drop compensation)

# ============================================================================
# SHOOTING PARAMETERS
# ============================================================================
# RPM Lookup Table: distance (inches) -> flywheel RPM
# These are STARTING VALUES - tune empirically!
RPM_LOOKUP_TABLE = {
    24: 4000,   # Close shot
    30: 4500,
    36: 5000,   # Medium range
    42: 5500,
    48: 6000,   # Far shot
    54: 6500,
    60: 7000,   # Max range
    66: 7500,   # Safety margin
    72: 8000    # Back wall (unlikely to use)
}

# Flywheel idle speed (RPM) when not shooting
FLYWHEEL_IDLE_RPM = 3500

# Alignment tolerances (relaxed for speed)
LAZY_SUSAN_ANGLE_TOLERANCE = 3.0   # degrees
FLYWHEEL_RPM_TOLERANCE = 3.0       # percent

# ============================================================================
# BASKET POSITIONS
# ============================================================================
# Lazy susan angles for each basket (degrees from center)
# Center = 0°, positive = clockwise, negative = counter-clockwise
# Adjust these based on your robot's geometry
BASKET_ANGLES = {
    'CENTER': 0.0,
    'LEFT': -12.0,   # ~15" offset at typical shooting distance
    'RIGHT': 12.0
}

# Default hunting position (point at center when idle)
IDLE_LAZY_SUSAN_ANGLE = BASKET_ANGLES['CENTER']

# ============================================================================
# ROBOT PHYSICAL PARAMETERS
# ============================================================================
BALL_COUNT = 10  # Starting number of balls

# Shooter parameters (for trajectory calculation if needed)
SHOOTER_HEIGHT_INCHES = 18.0      # Same as hoop
HOOP_HEIGHT_INCHES = 18.0
LAUNCH_ANGLE_DEGREES = 15.0
FLYWHEEL_RADIUS_MM = 20.0         # 40mm diameter / 2

# Drive wheel parameters
WHEEL_DIAMETER_INCHES = 2.7559055  # Drive wheel diameter
WHEEL_RADIUS_INCHES = WHEEL_DIAMETER_INCHES / 2
WHEEL_DIAMETER_MM = WHEEL_DIAMETER_INCHES * 25.4  # ~70mm
WHEELBASE_MM = 141.0               # Distance between wheel centers (middle to middle)
ENCODER_COUNTS_PER_REV = 1440      # Encoder resolution

# Calculated rotation parameters
# For 90 deg robot rotation:
# Arc traveled by each wheel = (90/360) * pi * wheelbase = (pi/4) * wheelbase
# Wheel revolutions = arc / (pi * wheel_diameter) = wheelbase / (4 * wheel_diameter)
# Encoder counts per wheel = revolutions * 1440
import math
ENCODER_COUNTS_90_DEG = int((WHEELBASE_MM / (4 * WHEEL_DIAMETER_MM)) * ENCODER_COUNTS_PER_REV)
# For differential (right-left)/2: when right=+726, left=-726, rotation = 726
ENCODER_ROTATION_90_DEG = ENCODER_COUNTS_90_DEG  # Full count, differential formula handles it

# ============================================================================
# LOCALIZATION PARAMETERS
# ============================================================================
# Arena dimensions (from competition specs)
ARENA_WIDTH = 72.0                # inches (6 feet)
ARENA_LENGTH = 72.0               # inches (6 feet)

# Basket positions (all at x = ARENA_LENGTH, on centerline/backboard)
BASKET_POSITIONS = {
    'LEFT': (72.0, 15.0),         # (x, y) in inches
    'CENTER': (72.0, 36.0),       # Center of arena
    'RIGHT': (72.0, 57.0)         # 72 - 15 = 57
}

# Rotation speed for scanning (max 127 due to signed byte limit)
ROTATION_SPEED = 127              # Rotation speed for localization scan (max signed byte)

# Scan parameters
SCAN_SAMPLES = 72                 # More samples for better wall detection (5° resolution)
SCAN_DELAY_MS = 50                # Faster scanning (50ms × 72 = 3.6s total)

# Wall detection threshold
MIN_VALID_DISTANCE = 12           # Minimum valid distance (inches)
MAX_VALID_DISTANCE = 80           # Maximum valid distance (inches)

# Position calculation
USE_TRIANGULATION = True          # Calculate position from 4 walls instead of driving
STAY_IN_PLACE = True              # Don't drive - just rotate to align

# Shooting strategy
RETURN_TO_CENTER_AFTER_SHOT = True  # Return lazy susan to 0° after each shot
                                     # Ensures consistent ready position

# ============================================================================
# SENSOR PARAMETERS
# ============================================================================
# IR sensor configuration (Vishay TSOP34156)
# 3 sensors for left, center, right beacon detection
IR_DETECTION_THRESHOLD = 0.5  # Seconds of stable detection before accepting

# IR Filter Parameters (Hysteresis)
IR_FILTER_SAMPLES = 10        # Number of samples to keep in history
IR_FILTER_THRESHOLD_HIGH = 6  # >60% (6/10) to activate (STOP)
IR_FILTER_THRESHOLD_LOW = 4   # <40% (4/10) to deactivate (START)

# Ultrasonic sensor
ULTRASONIC_MIN_DISTANCE = 12  # inches (too close to shoot)
ULTRASONIC_MAX_DISTANCE = 72  # inches (back wall)
ULTRASONIC_SAMPLES = 3        # Average multiple readings

# ============================================================================
# DEBUGGING / LOGGING
# ============================================================================
DEBUG_MODE = True
LOG_SHOT_TIMING = True
LOG_TELEMETRY = False  # Set True for verbose sensor data logging

# ============================================================================
# COMPETITION SETTINGS
# ============================================================================
GAME_DURATION_SECONDS = 120  # 2 minutes
ENABLE_GAME_TIMER = True
