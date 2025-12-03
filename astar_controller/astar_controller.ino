#include <AStar32U4Motors.h>
#include <Encoder.h>
#include <AccelStepper.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Encoders
Encoder encoderRight(A0, A1);
Encoder encoderLeft(A2, A3);

// Motors
AStar32U4Motors motors;

// Loader Stepper
// STEP = A4, DIR = A5
AccelStepper loader(AccelStepper::DRIVER, A4, A5);
const int LOADER_ENABLE_PIN = 12;

// Ultrasonic (Optional, for telemetry)
const int TRIG_PIN = 4;
const int ECHO_PIN = 8;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Communication
const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];
boolean newData = false;

// Command State
int cmd_motorSpeed = 0;      // Desired panning speed
int cmd_irState = 1;         // 0 = Detected, 1 = Not Detected
int cmd_shoot = 0;           // 1 = Shoot

// Panning Logic State
unsigned long stopTimestamp = 0;
const unsigned long MIN_STOP_DURATION = 500;

// Encoder Panning State
long rotationTarget = 0;
const long ENCODER_PAN_RANGE = 363;  // ~45 degrees
const long SAFETY_LIMIT = 800;
int currentDirection = 0; // 1 = Right, -1 = Left

// Loader State
bool isShooting = false;
long loaderTarget = 0;
const long STEPS_PER_90_DEG = 50; // Adjust based on microstepping (200 * 1/4 / 4 = 12.5? Need to calibrate)
// Assuming 200 steps/rev, 1/16 microstepping = 3200 steps/rev
// 90 deg = 800 steps.
// Let's use a safe default and assume user calibrates or we use the config value.
// config.h said LOADER_MICROSTEPS 16.
const long LOADER_STEPS_90 = 800; 

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);

    // Encoders
    encoderRight.write(0);
    encoderLeft.write(0);

    // Loader
    // pinMode(LOADER_ENABLE_PIN, OUTPUT);
    // digitalWrite(LOADER_ENABLE_PIN, LOW); // Enable
    loader.setMaxSpeed(10000);
    loader.setAcceleration(5000);

    // Ultrasonic
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    Serial.println("A-Star Ready (ASCII Protocol)");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    recvWithStartEndMarkers();
    
    if (newData == true) {
        strcpy(tempChar, receivedChars);
        parseData();
        newData = false;
    }
    
    commandMotors();
    runLoader();
}

// ============================================================================
// COMMUNICATION PARSING
// ============================================================================

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

void parseData() {
    char *strtokIndexer;
    
    // Format: <motorSpeed, irState, shootCommand>
    
    // 1. Motor Speed
    strtokIndexer = strtok(tempChar, ",");
    if (strtokIndexer != NULL) {
        cmd_motorSpeed = atoi(strtokIndexer);
    }
    
    // 2. IR State
    strtokIndexer = strtok(NULL, ",");
    if (strtokIndexer != NULL) {
        cmd_irState = atoi(strtokIndexer);
    }
    
    // 3. Shoot Command
    strtokIndexer = strtok(NULL, ",");
    if (strtokIndexer != NULL) {
        cmd_shoot = atoi(strtokIndexer);
        if (cmd_shoot == 1) {
            triggerShoot();
        }
    }
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void commandMotors() {
    unsigned long currentTime = millis();
    
    // Differential Encoder Calculation
    long encoderRight_val = encoderRight.read();
    long encoderLeft_val = encoderLeft.read();
    long currentRotation = (encoderRight_val - encoderLeft_val) / 2;
    
    // IR Detection Logic
    if (cmd_irState == 0) { // DETECTED
        motors.setM1Speed(0);
        motors.setM2Speed(0);
        stopTimestamp = currentTime;
    }
    else { // NOT DETECTED
        // Wait for minimum stop duration
        if (currentTime - stopTimestamp >= MIN_STOP_DURATION) {
            
            // Panning Logic (from IRscan.ino)
            
            // Safety Limits
            if (currentRotation > SAFETY_LIMIT && currentDirection == 1) {
                currentDirection = -1;
                rotationTarget = -ENCODER_PAN_RANGE;
            } else if (currentRotation < -SAFETY_LIMIT && currentDirection == -1) {
                currentDirection = 1;
                rotationTarget = ENCODER_PAN_RANGE;
            }
            
            // Initialization
            if (currentDirection == 0) {
                currentDirection = -1;
                rotationTarget = -ENCODER_PAN_RANGE;
            }
            
            // Reversal Logic
            if (currentDirection < 0 && currentRotation <= rotationTarget) {
                currentDirection = 1;
                rotationTarget = ENCODER_PAN_RANGE;
            } else if (currentDirection > 0 && currentRotation >= rotationTarget) {
                currentDirection = -1;
                rotationTarget = -ENCODER_PAN_RANGE;
            }
            
            // Drive Motors
            int speed = abs(cmd_motorSpeed);
            if (currentDirection > 0) {
                // Rotate Right
                motors.setM1Speed(-speed);
                motors.setM2Speed(speed);
            } else {
                // Rotate Left
                motors.setM1Speed(speed);
                motors.setM2Speed(-speed);
            }
            
        } else {
            // Still waiting
            motors.setM1Speed(0);
            motors.setM2Speed(0);
        }
    }
}

// ============================================================================
// LOADER CONTROL
// ============================================================================

void triggerShoot() {
    if (loader.distanceToGo() == 0) {
        loader.move(LOADER_STEPS_90);
    }
}

void runLoader() {
    loader.run();
}
