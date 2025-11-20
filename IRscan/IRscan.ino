#include <AStar32U4Motors.h>
AStar32U4Motors m;

const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars];

// Global variables
int leftSpeed = 0;
int rightSpeed = 0;
boolean newData = false;

void parseData(){
  char *strtokIndexer;
  
  // Parse: <left_speed,right_speed>
  strtokIndexer = strtok(tempChar, ",");
  if (strtokIndexer != NULL) {
    leftSpeed = atoi(strtokIndexer);
  }
  
  strtokIndexer = strtok(NULL, ",");
  if (strtokIndexer != NULL) {
    rightSpeed = atoi(strtokIndexer);
  }
}

void commandMotors(){
    // Set differential drive speeds for rotation
    // Left motor controls left wheel, right motor controls right wheel
    m.setM1Speed(leftSpeed);
    m.setM2Speed(rightSpeed);
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
