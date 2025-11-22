#include <AStar32U4Motors.h>
#include <Encoder.h>

AStar32U4Motors m;

// Encoders: Right wheel = A0, A1 | Left wheel = A2, A3
// Resolution: 1440 counts per wheel revolution
Encoder encoderRight(A0, A1);  // Right wheel encoder
Encoder encoderLeft(A2, A3);   // Left wheel encoder

// Ultrasonic sensor pins
#define ULTRASONIC_TRIG         15
#define ULTRASONIC_ECHO         16
#define ULTRASONIC_TIMEOUT_US   30000  // 30ms timeout (~5m max range)
#define SOUND_SPEED_CM_US       0.0343 // Speed of sound: 343 m/s = 0.0343 cm/µs

const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];

// Global variables
int motorSpeed = 0;      // Desired motor speed from Python (+75, -75, or 0)
int irValue = 1;         // IR state: 0 = detected, 1 = not detected
int turnFlag = 0;        // Turn direction flag from Python: 0=normal, 1=force right, -1=force left
boolean newData = false;
unsigned long stopTimestamp = 0;  // Track when we stopped
const unsigned long MIN_STOP_DURATION = 500;  // Minimum 500ms stop time
int ultrasonicDistance = 0;  // Current ultrasonic distance in cm

// Encoder-based panning
long rotationTarget = 0;     // Target rotation position
// ENCODER_PAN_RANGE: Set for 45 deg sweep each direction (90 deg total centered at 0)
// Boundaries: +363 (right limit) and -363 (left limit)
const long ENCODER_PAN_RANGE = 363;  // Rotation value for 45 deg sweep
const long SAFETY_LIMIT = 800;       // Hard limit to catch motor stalls/runaway
int currentDirection = 0;            // Track current panning direction: 1=right, -1=left

void parseData(){
  char *strtokIndexer;
  
  // Parse: <motor_speed,ir_state,turn_flag>
  strtokIndexer = strtok(tempChar, ",");
  if (strtokIndexer != NULL) {
    motorSpeed = atoi(strtokIndexer);
  }
  
  strtokIndexer = strtok(NULL, ",");
  if (strtokIndexer != NULL) {
    irValue = atoi(strtokIndexer);
  }
  
  strtokIndexer = strtok(NULL, ",");
  if (strtokIndexer != NULL) {
    turnFlag = atoi(strtokIndexer);
  }
}

// Read ultrasonic distance in cm
int readUltrasonic() {
  // Send trigger pulse
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Read echo pulse
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
  
  // Calculate distance in cm
  if (duration == 0) {
    return 999;  // Timeout = no object detected
  }
  
  int distance = (duration * SOUND_SPEED_CM_US) / 2;
  return distance;
}

void commandMotors(){
    unsigned long currentTime = millis();
    
    // Calculate rotation from differential encoder readings
    // For in-place rotation: right moves forward, left moves backward
    // Rotation = (encoderRight - encoderLeft) / 2
    long encoderRight_val = encoderRight.read();
    long encoderLeft_val = encoderLeft.read();
    long currentRotation = (encoderRight_val - encoderLeft_val) / 2;
    
    // Read ultrasonic distance
    static unsigned long lastUltrasonicTime = 0;
    if(currentTime - lastUltrasonicTime >= 100) {  // Read every 100ms
        ultrasonicDistance = readUltrasonic();
        lastUltrasonicTime = currentTime;
    }
    
    // Debug output every 500ms
    static unsigned long lastDebugTime = 0;
    if(currentTime - lastDebugTime >= 500) {
        Serial.print("Dist:");
        Serial.print(ultrasonicDistance);
        Serial.print("cm Turn:");
        Serial.print(turnFlag);
        Serial.print(" Rot:");
        Serial.print(currentRotation);
        Serial.print(" Dir:");
        Serial.println(currentDirection);
        lastDebugTime = currentTime;
    }
    
    // Send ultrasonic distance to Python (format: D:123)
    static unsigned long lastSendTime = 0;
    if(currentTime - lastSendTime >= 100) {  // Send every 100ms
        Serial.print("D:");
        Serial.println(ultrasonicDistance);
        lastSendTime = currentTime;
    }
    
    if(irValue == 0){  // Beacon DETECTED (stop)
        m.setM1Speed(0);
        m.setM2Speed(0);
        stopTimestamp = currentTime;
    }
    else {  // Beacon NOT detected
        // Only allow movement if we've been stopped for at least MIN_STOP_DURATION
        if(currentTime - stopTimestamp >= MIN_STOP_DURATION){
            
            // Initialize direction on first run
            if(currentDirection == 0) {
                currentDirection = -1;  // Start panning LEFT
                Serial.println("*** INIT: Starting LEFT");
            }
            
            // Check Python's turn_flag for wall-based direction control
            if(turnFlag == 1 && currentDirection != 1) {
                // Python detected wall on left, turn RIGHT
                currentDirection = 1;
                Serial.println("*** WALL DETECT: Now going RIGHT");
            } 
            else if(turnFlag == -1 && currentDirection != -1) {
                // Python detected wall on right, turn LEFT
                currentDirection = -1;
                Serial.println("*** WALL DETECT: Now going LEFT");
            }
            
            // Extreme safety fallback based on encoder (only if ultrasonic fails completely)
            // Increased limit to 5000 so it doesn't interfere with normal ultrasonic operation
            const long EXTREME_SAFETY_LIMIT = 5000;
            if(currentRotation > EXTREME_SAFETY_LIMIT && currentDirection == 1) {
                currentDirection = -1;
                Serial.println("*** EXTREME SAFETY: > Limit, forcing LEFT");
            }
            else if(currentRotation < -EXTREME_SAFETY_LIMIT && currentDirection == -1) {
                currentDirection = 1;
                Serial.println("*** EXTREME SAFETY: < Limit, forcing RIGHT");
            }
            
            // Apply current direction to motor speed (use abs of motorSpeed from Python)
            // For in-place rotation: motors must spin in OPPOSITE directions
            // M1 = Right motor, M2 = Left motor
            int actualSpeed = abs(motorSpeed);
            
            if(currentDirection > 0) {
                // Rotate right (clockwise): right backward, left forward
                m.setM1Speed(-actualSpeed);
                m.setM2Speed(actualSpeed);
            } else {
                // Rotate left (counter-clockwise): right forward, left backward  
                m.setM1Speed(actualSpeed);
                m.setM2Speed(-actualSpeed);
            }

        } else {
            // Still in minimum stop period - keep motors stopped
            m.setM1Speed(0);
            m.setM2Speed(0);
        }
    }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
                                                               
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
                                                             
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0';
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void setup() {
    Serial.begin(115200);
    encoderRight.write(0);  // Reset right encoder
    encoderLeft.write(0);   // Reset left encoder
    
    // Setup ultrasonic sensor
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    digitalWrite(ULTRASONIC_TRIG, LOW);
}

void loop() {
    recvWithStartEndMarkers();
    
    if (newData == true) {
        strcpy(tempChar, receivedChars);
        parseData();
        newData = false;
    }
    
    commandMotors();
}
