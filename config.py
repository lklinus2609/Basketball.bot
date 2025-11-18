"""
Configuration file for Basketball Robot
All tunable parameters in one place for easy adjustment
"""

# ============================================================================
# COMMUNICATION SETTINGS
# ============================================================================
SERIAL_PORT = '/dev/ttyAMA0'  # RPI GPIO serial port (adjust if using USB)
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
STABILIZATION_TIME = 100        # How long to hold stable before shooting
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

# ============================================================================
# LOCALIZATION PARAMETERS
# ============================================================================
# Target shooting distance (inches from backboard)
OPTIMAL_SHOOTING_DISTANCE = 36    # Sweet spot for accuracy

# Distance tolerance for positioning
DISTANCE_TOLERANCE = 3            # ±3 inches is acceptable

# Rotation speed for scanning (0-255)
ROTATION_SPEED = 100              # Slow rotation for scanning

# Drive speed for positioning (0-255)
DRIVE_SPEED_SLOW = 80             # Slow approach
DRIVE_SPEED_FAST = 150            # Quick repositioning

# Scan parameters
SCAN_SAMPLES = 36                 # Take 36 samples during 360° rotation
SCAN_DELAY_MS = 100               # Delay between samples

# Wall detection threshold
MIN_VALID_DISTANCE = 12           # Minimum valid distance (inches)
MAX_VALID_DISTANCE = 72           # Maximum valid distance (inches)

# ============================================================================
# SENSOR PARAMETERS
# ============================================================================
# IR sensor configuration (Vishay TSOP34156)
# 3 sensors for left, center, right beacon detection
IR_DETECTION_THRESHOLD = 0.5  # Seconds of stable detection before accepting

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
