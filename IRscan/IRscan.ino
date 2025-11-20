#include <AStar32U4Motors.h>
#include <Encoder.h>

AStar32U4Motors m;

// Encoders: Right wheel = A0,A1 | Left wheel = A2,A3
// Resolution: 1440 counts per wheel revolution
Encoder encoderRight(0, 1);  // Right wheel encoder (A0, A1) for rotation tracking

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
long encoderTarget = 0;      // Target encoder position for this direction
// ENCODER_PAN_RANGE: Tune this value to achieve desired 90° sweep
// Start with 360 (~1/4 wheel revolution) and adjust based on actual robot behavior
// Increase value = wider sweep, Decrease value = narrower sweep
const long ENCODER_PAN_RANGE = 360;  // Encoder ticks for ~45° robot rotation (half of 90° sweep)
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
    long currentEncoder = encoderRight.read();
    
    if(irValue == 0){  // Beacon DETECTED (stop)
        m.setM1Speed(0);
        m.setM2Speed(0);
        stopTimestamp = currentTime;
    }
    else {  // Beacon NOT detected
        // Only allow movement if we've been stopped for at least MIN_STOP_DURATION
        if(currentTime - stopTimestamp >= MIN_STOP_DURATION){
            // Check if we need to reverse direction based on encoder position
            if(motorSpeed > 0 && currentEncoder >= encoderTarget) {
                // Hit right limit, reverse to left
                encoderTarget = currentEncoder - ENCODER_PAN_RANGE;
                currentDirection = -1;
            } 
            else if(motorSpeed < 0 && currentEncoder <= encoderTarget) {
                // Hit left limit, reverse to right
                encoderTarget = currentEncoder + ENCODER_PAN_RANGE;
                currentDirection = 1;
            }
            
            // Use motor speed from Python, but apply current direction
            int actualSpeed = (currentDirection == 0) ? motorSpeed : (currentDirection > 0 ? abs(motorSpeed) : -abs(motorSpeed));
            
            // Initialize direction on first run
            if(currentDirection == 0) {
                currentDirection = (motorSpeed > 0) ? 1 : -1;
                encoderTarget = currentEncoder + (currentDirection * ENCODER_PAN_RANGE);
            }
            
            m.setM1Speed(actualSpeed);
            m.setM2Speed(actualSpeed);
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
    encoderRight.write(0);  // Reset encoder position
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
