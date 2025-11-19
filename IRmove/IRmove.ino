#include <AStar32U4Motors.h>
#include <Encoder.h>
AStar32U4Motors m; //read t   he documentation of this library to understand what functions to use to drive the motors and how to use them

const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars]; // temporary array used for parsing


//motor command variables
int irValue = 1;
boolean newData = false;


double interval = 3000;
boolean cooldown = false;
unsigned long previousMillis = 0;


void setup() {
    Serial.begin(115200);
    Serial.println("<Arduino is ready>");
    pinMode(3,OUTPUT);
}

void loop() {
    recvWithStartEndMarkers();
    unsigned long currentMillis = millis();
    if(cooldown && (currentMillis - previousMillis >= interval)){
          cooldown = false;
          digitalWrite(3,LOW);
      }
    if (newData == true) {
      strcpy(tempChar, receivedChars);
        if(!cooldown){
          parseData(); 
          if(irValue==0){
              previousMillis = currentMillis;
              cooldown = true;
              digitalWrite(3,HIGH);
            }
        }
      newData = false;

      
                   
    }

    commandMotors(); //we want this to happen outside of our newdata=true loop so it is never blocked
}


//======================================================


void parseData(){
  char *strtokIndexer; //doing char * allows strtok to increment across my string properly frankly im not sure why... something to do with pointers that I dont expect students to understand
  strtokIndexer = strtok(tempChar,","); //sets strtokIndexer = to everything up to the first comma in tempChar /0 //this line is broken
  irValue = atoi(strtokIndexer); //converts strtokIndexer into a int
  
  //now that we have extracted the data from the Rpi as floats, we can use them to command actuators somewhere else in the code
  
}

//==========================================


//=======================================

void commandMotors(){
    if(irValue){
        m.setM1Speed(75);
        m.setM2Speed(75);
    }else{
      m.setM1Speed(0);
      m.setM2Speed(0);
    }
}


//=========================================================


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
                receivedChars[ndx] = '\0'; // terminates the string, frankly unsure why I need this
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
