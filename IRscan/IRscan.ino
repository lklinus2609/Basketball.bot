#include <AStar32U4Motors.h>
#include <Encoder.h>

AStar32U4Motors m;

// Encoders: Right wheel = A0, A1 | Left wheel = A2, A3
// Resolution: 1440 counts per wheel revolution
Encoder encoderRight(A0, A1);  // Right wheel encoder
Encoder encoderLeft(A2, A3);   // Left wheel encoder

const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];

// Global variables
int motorSpeed = 0;      // Desired motor speed from Python (+75, -75, or 0)
int irValue = 1;         // IR state: 0 = detected, 1 = not detected
boolean newData = false;
unsigned long stopTimestamp = 0;  // Track when we stopped
const unsigned long MIN_STOP_DURATION = 500;  // Minimum 500ms stop time

// Encoder-based panning
long rotationTarget = 0;     // Target rotation position
// ENCODER_PAN_RANGE: Tune this value to achieve desired 90 deg sweep
// Start with 360 (~1/4 wheel revolution) and adjust based on actual robot behavior
// Increase value = wider sweep, Decrease value = narrower sweep
const long ENCODER_PAN_RANGE = 360;  // Encoder ticks for ~45 deg robot rotation (half of 90 deg sweep)
int currentDirection = 0;    // Track current panning direction: 1=right, -1=left

void parseData(){
  char *strtokIndexer;
  
  // Parse: <motor_speed,ir_state>
  strtokIndexer = strtok(tempChar, ",");
  if (strtokIndexer != NULL) {
    motorSpeed = atoi(strtokIndexer);
  }
  
  strtokIndexer = strtok(NULL, ",");
  if (strtokIndexer != NULL) {
    irValue = atoi(strtokIndexer);
  }
}

void commandMotors(){
    unsigned long currentTime = millis();
    
    // Calculate rotation from differential encoder readings
    // For in-place rotation: right moves forward, left moves backward
    // Rotation = (encoderRight - encoderLeft) / 2
    long encoderRight_val = encoderRight.read();
    long encoderLeft_val = encoderLeft.read();
    long currentRotation = (encoderRight_val - encoderLeft_val) / 2;
    
    // Debug output every 500ms
    static unsigned long lastDebugTime = 0;
    if(currentTime - lastDebugTime >= 500) {
        Serial.print("R:");
        Serial.print(encoderRight_val);
        Serial.print(" L:");
        Serial.print(encoderLeft_val);
        Serial.print(" Rot:");
        Serial.print(currentRotation);
        Serial.print(" Tgt:");
        Serial.print(rotationTarget);
        Serial.print(" Dir:");
        Serial.println(currentDirection);
        lastDebugTime = currentTime;
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
                currentDirection = -1;  // Start panning LEFT (negative = clockwise based on wiring)
                rotationTarget = currentRotation - ENCODER_PAN_RANGE;
                Serial.println("*** INIT: Starting LEFT");
            }
            
            // Check if we need to reverse direction based on rotation position
            if(currentDirection < 0 && currentRotation <= rotationTarget) {
                // Hit left limit, reverse to right
                currentDirection = 1;
                rotationTarget = currentRotation + ENCODER_PAN_RANGE;
                Serial.println("*** REVERSE: Now going RIGHT");
            } 
            else if(currentDirection > 0 && currentRotation >= rotationTarget) {
                // Hit right limit, reverse to left
                currentDirection = -1;
                rotationTarget = currentRotation - ENCODER_PAN_RANGE;
                Serial.println("*** REVERSE: Now going LEFT");
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
