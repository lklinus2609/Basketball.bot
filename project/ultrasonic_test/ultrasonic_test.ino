/**
 * Ultrasonic 360° Scan Test for A-Star 32U4
 * Rotates the robot using drive motors and scans for the furthest wall
 * 
 * Hardware:
 * - Ultrasonic TRIG pin: 15, ECHO pin: 16
 * - Drive motors: M1 (right), M2 (left) using AStar32U4Motors library
 * 
 * Operation:
 * - Press 's' in Serial Monitor to start a scan
 * - Robot rotates in place 360° and finds the furthest wall
 */

#include <AStar32U4Motors.h>

// Pin definitions - Ultrasonic
#define ULTRASONIC_TRIG         15
#define ULTRASONIC_ECHO         16

// Constants - Ultrasonic
#define ULTRASONIC_TIMEOUT_US   30000  // 30ms timeout (~5m max range)
#define SOUND_SPEED_CM_US       0.0343 // Speed of sound: 343 m/s = 0.0343 cm/µs

// Constants - Rotation
#define ROTATION_SPEED          75     // Motor speed for rotation (adjust as needed)
#define ROTATION_TIME_PER_10DEG 150     // Milliseconds to rotate 10° (CALIBRATE THIS!)

// Scan parameters
#define SCAN_STEP_DEGREES       10      // Take measurement every 10 degrees
#define NUM_MEASUREMENTS        36      // 360/10 = 36 measurements

// Global variables
AStar32U4Motors motors;
bool scanning = false;
uint16_t scan_distances[NUM_MEASUREMENTS];
int scan_index = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("===========================================");
  Serial.println("Ultrasonic 360° Scan Test");
  Serial.println("===========================================");
  Serial.println("This program rotates the robot in place");
  Serial.println("and finds the furthest wall using ultrasonic.");
  Serial.println("===========================================\n");
  
  // Setup ultrasonic sensor
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Stop motors on startup
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  
  Serial.println("Commands:");
  Serial.println("  's' - Start 360° scan");
  Serial.println("  't' - Test single ultrasonic reading");
  Serial.println("  'c' - Calibrate rotation timing");
  Serial.println("===========================================\n");
  
  delay(1000);
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 's' || cmd == 'S') {
      startScan();
    } else if (cmd == 't' || cmd == 'T') {
      testSingleReading();
    } else if (cmd == 'c' || cmd == 'C') {
      calibrateRotation();
    }
  }
}

/**
 * Start a 360° scan
 */
void startScan() {
  Serial.println("\n>>> Starting 360° scan...");
  Serial.println("Scanning every " + String(SCAN_STEP_DEGREES) + " degrees\n");
  
  // Take measurements at each angle
  for (int i = 0; i < NUM_MEASUREMENTS; i++) {
    // Take measurement
    delay(200); // Let robot settle
    uint16_t distance = readUltrasonic();
    scan_distances[i] = distance;
    
    int current_angle = i * SCAN_STEP_DEGREES;
    Serial.print("Angle ");
    Serial.print(current_angle);
    Serial.print("°: ");
    Serial.print(distance);
    Serial.println(" cm");
    
    // Rotate to next position (unless this is the last measurement)
    if (i < NUM_MEASUREMENTS - 1) {
      rotateRobot(SCAN_STEP_DEGREES);
    }
  }
  
  finishScan();
}

/**
 * Rotate robot in place by specified degrees (clockwise)
 * Uses motor timing - NEEDS CALIBRATION!
 */
void rotateRobot(int degrees) {
  // Calculate rotation time
  unsigned long rotate_time = (ROTATION_TIME_PER_10DEG * degrees) / 10;
  
  // Rotate clockwise: M1 forward, M2 backward
  motors.setM1Speed(ROTATION_SPEED);   // Right motor forward
  motors.setM2Speed(ROTATION_SPEED);  // Left motor backward
  
  delay(rotate_time);
  
  // Stop
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}

/**
 * Calibrate rotation timing
 */
void calibrateRotation() {
  Serial.println("\n>>> Calibration Mode");
  Serial.println("Robot will rotate clockwise.");
  Serial.println("Measure how far it rotates and adjust ROTATION_TIME_PER_10DEG");
  Serial.println("Current setting: " + String(ROTATION_TIME_PER_10DEG) + " ms per 10°\n");
  
  Serial.println("Rotating for 1 second...");
  motors.setM1Speed(ROTATION_SPEED);
  motors.setM2Speed(-ROTATION_SPEED);
  delay(1000);
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  
  Serial.println("Rotation stopped. Measure the angle rotated.");
  Serial.println("If it rotated X degrees, set ROTATION_TIME_PER_10DEG = " + String(1000) + " * 10 / X\n");
}

/**
 * Finish the scan and analyze results
 */
void finishScan() {
  scanning = false;
  
  Serial.println("\n===========================================");
  Serial.println("Scan Complete!");
  Serial.println("===========================================\n");
  
  // Find furthest wall
  uint16_t max_distance = 0;
  int max_angle = 0;
  
  for (int i = 0; i < NUM_MEASUREMENTS; i++) {
    if (scan_distances[i] > max_distance) {
      max_distance = scan_distances[i];
      max_angle = i * SCAN_STEP_DEGREES;
    }
  }
  
  Serial.println("FURTHEST WALL:");
  Serial.print("  Angle: ");
  Serial.print(max_angle);
  Serial.println("°");
  Serial.print("  Distance: ");
  Serial.print(max_distance);
  Serial.println(" cm");
  
  // Print all measurements
  Serial.println("\nAll measurements:");
  for (int i = 0; i < NUM_MEASUREMENTS; i++) {
    int angle = i * SCAN_STEP_DEGREES;
    Serial.print("  ");
    Serial.print(angle);
    Serial.print("°: ");
    Serial.print(scan_distances[i]);
    Serial.print(" cm");
    
    // Mark the furthest
    if (i * SCAN_STEP_DEGREES == max_angle) {
      Serial.print(" <-- FURTHEST");
    }
    Serial.println();
  }
  
  Serial.println("\n===========================================");
  Serial.println("Ready for next command ('s' to scan again)");
  Serial.println("===========================================\n");
}

/**
 * Test a single ultrasonic reading at current position
 */
void testSingleReading() {
  uint16_t distance_cm = readUltrasonic();
  float distance_inches = distance_cm / 2.54;
  
  Serial.print("\nSingle reading: ");
  Serial.print(distance_cm);
  Serial.print(" cm (");
  Serial.print(distance_inches, 1);
  Serial.println(" inches)\n");
}

/**
 * Read ultrasonic distance sensor
 * Returns: Distance in centimeters
 */
uint16_t readUltrasonic() {
  // Send trigger pulse
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Measure echo pulse duration
  unsigned long duration = pulseIn(ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
  
  // Calculate distance in cm
  // Distance = (duration * speed_of_sound) / 2
  uint16_t distance_cm = (uint16_t)((duration * SOUND_SPEED_CM_US) / 2.0);
  
  // Return reading (0 if out of range)
  if (distance_cm > 400 || duration == 0) {
    return 0; // Out of range or no echo
  }
  
  return distance_cm;
}

