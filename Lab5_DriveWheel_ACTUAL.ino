
#include <AStar32U4Motors.h>
#include <Encoder.h>


AStar32U4Motors m; //read the documentation of this library to understand what functions to use to drive the motors and how to use them

#define PI 3.141592653589

int leftMotor; // COMMANDED MOTOR SPEEDS
int rightMotor;

double leftMotorMax = 25.25; 
double rightMotorMax = 20.44;

const int encoderRightPinA = A4;//a1
const int encoderRightPinB = A3;//a0

const int encoderLeftPinA = A1; 
const int encoderLeftPinB = A0;

Encoder encoderRight(encoderRightPinA,encoderRightPinB);
Encoder encoderLeft(encoderLeftPinA,encoderLeftPinB);

int encoderResolution = 1440; // counts per rev
double d = 2.7559055; //wheel diameter in inches

int posLeftCount = 0;
int posRightCount = 0;
int posLeftCountLast = 0;
int posRightCountLast = 0;
double posLeftRad = 0.0; // this will need to be converted to rad/sec
double posRightRad = 0.0; // this will need to be converted to rad/sec
double posLeftRadLast = 0.0;
double posRightRadLast = 0.0; 
double velLeft = 0; // this will be omegaLeft*d/2;
double velRight = 0; // this will be omegaRight*d/2 will be in inches per sec;
double newVelLeft = 0; // this will be omegaLeft*d/2;
double newVelRight = 0; // this will be omegaRight*d/2 will be in inches per sec;

// MOTOR LOOP CONSTANTS
double interval = 5.0; // 5 ms means 200Hz loop
unsigned long previousMillis = 0;
unsigned long priorTimeL,priorTimeR; // We need to keep track of time for each PID controller separately
double lastSpeedErrorL,lastSpeedErrorR; //same with error
double cumErrorL, cumErrorR;
double maxErr = 20; // chosen arbitrarily for now, students can tune. 
double desVelL = 0; // will be in inches per sec
double desVelR = 0;
int count = 0;
int velocities[] = {12,13,14,15};
int i = 0;

// PID CONSTANTS
// LEFT MOTOR - you need to find values. FYI I found good responses with Kp ~ 10x bigger than Ki, and ~3x bigger than Kd. My biggest value was <2.
double kpL = 3.0;
double kiL = 1.0;
double kdL = 1.0;
// Right MOTOR - assumes we need to tune them differently
double kpR = 3.0;
double kiR = 1.0;
double kdR = 1.0;                                                                                                                                                                                              ;

//=====================================================

void setup() {
  Serial.begin(115200);
    m.setM1Speed(0);  // MAX IS 400 FYI. You should set this first to see max speed in in/s after you convert the values
    m.setM2Speed(0);  
 

}

void loop() {

   unsigned long currentMillis = millis();

     posRightCount = encoderRight.read();
     posLeftCount = encoderLeft.read();

//     Serial.print("encoderR:");
//     Serial.println(posRightCount);
//     Serial.print("encoderL:");
//     Serial.println(posLeftCount);

   if (currentMillis - previousMillis >= interval){
      previousMillis = currentMillis;
      desVelL = velocities[i]; // will be in inches per sec
      desVelR = velocities[i];
      Serial.print(" i ");
      Serial.print(i);
      Serial.print(" desVelL ");
      Serial.print(velocities[i]);
      Serial.print(" desVelR ");
      Serial.print(velocities[i]);

     posRightRad = 2*PI*(posRightCount/1440); // Write expression to get Rad/sec. Pi is defined above FYI.
     posLeftRad = 2*PI*(posLeftCount/1440); // Same - Rad

     double deltaRightCount = posRightCount-posRightCountLast;
     double deltaRight = (deltaRightCount/1440) * 2 * PI;
     double deltaLeftCount = posLeftCount-posLeftCountLast;
     double deltaLeft = (deltaLeftCount/1440) * 2 * PI;
//     Serial.print("deltaRight:");
//     Serial.println(deltaRight);
//     Serial.print("deltaLeft:");
//     Serial.println(deltaLeft);
   
     velRight = 1000*0.5*d*(deltaRight/(interval)); // Now convert to get m/sec (tangential velocity)
     velLeft = 1000*0.5*d*(deltaLeft/(interval)); // Same - in/s

//     Serial.println("===");
//     Serial.println("Last");
//     Serial.println(posRightCountLast);
//     Serial.println("New");
//     Serial.println(posRightRad);
//     Serial.println("in/s");
//     Serial.println(velRight);
     Serial.println(" ===");
     Serial.print(" VelRight ");
     Serial.print(velRight);
     Serial.print(" VelLeft ");
     Serial.print(velLeft);

     // HERE WILL DO PID AND CREATE NEW MOTOR COMMAND MAPPED TO -400-400 based on max. 
     // COMMENT THIS SECTION OUT TO FIND YOUR MOTORS MAX SPEEDS,
     boolean value1 = false;
     boolean value2 = false;
     boolean value3 = (count % 200 == 0);
     newVelRight = drivePIDR(velRight,&value1);
     newVelLeft = drivePIDL(velLeft,&value2);
     Serial.print(" Value1 ");
     Serial.print(value1);
     Serial.print(" Value2 ");
     Serial.print(value2);
     Serial.print(" Value3 ");
     Serial.print(value3);

      rightMotor = motorVelToSpeedCommand(newVelRight,rightMotorMax);
      leftMotor = motorVelToSpeedCommand(newVelLeft,leftMotorMax);
      Serial.print(" rightMotor ");
      Serial.print(rightMotor);
      Serial.print(" leftMotor ");
      Serial.println(leftMotor);
      if(value1 && value2 && value3){
        if(i==3){
          i = 0;  
        }else{
          i++;  
        }
      }
//        rightMotor = 400;
//        leftMotor = 400;
      /// COMMENT OUT TO HERE FOR FINDING MAX MOTOR SPEED AT 400, You need to add the print statements to get the max speed. 
      
     
     posRightCountLast = posRightCount;
     posLeftCountLast = posLeftCount;

     posLeftRadLast = posLeftRad;
     posRightRadLast = posRightRad;

       CommandMotors();
       count++;
   }
}

void CommandMotors(){  

  //read the documentation for the functions that drive the motors in the astar library

  m.setM1Speed(rightMotor);
  m.setM2Speed(leftMotor);
  //uncomment to drive motors
}

double drivePIDL(double curr, boolean *value){
    double rateError;
    double error;
    unsigned long currentTime;
    unsigned long elapsedTime;
  
    currentTime = millis();                               //get current time
    elapsedTime = (double)(currentTime - priorTimeL);     // compute elasped time for this control period

    error = desVelL - curr;
    if(error <= 1){
      *value = true;
    }// Error
    cumErrorL += error*elapsedTime;                       // Cumulative Error(since we add this outside the loop, needs to be unique to the motor controlled)

    // INTEGRAL WINDUP                                    // REMOVE WINDUP
    if(cumErrorL>maxErr)
    cumErrorL = maxErr;
    else if (cumErrorL<-1*maxErr)
      cumErrorL = -1*maxErr;

    rateError = (error-lastSpeedErrorL)/elapsedTime;      // Derivative Error

    double out = kpL*error+kiL*cumErrorL+kdL*rateError;   // PID output

    lastSpeedErrorL = error;                              // remember current error
    priorTimeL = currentTime;                             // remember current time
    return out;                                           // return the needed motor speed. 
}
double drivePIDR(double curr,boolean *value){
    double rateError;
    double error;
    unsigned long currentTime;
    unsigned long elapsedTime;
  
    currentTime = millis();                               //get current time
    elapsedTime = (double)(currentTime - priorTimeR);      // compute elasped time for this control period

    error = desVelR - curr;     
    if(error <= 1){
      *value = true;
    }// Error// Error
    cumErrorR += error*elapsedTime;                       // Cumulative Error(since we add this outside the loop, needs to be unique to the motor controlled)

    // INTEGRAL WINDUP                                    // REMOVE WINDUP
    if(cumErrorR>maxErr)
    cumErrorR = maxErr;
    else if (cumErrorR<-1*maxErr)
      cumErrorR = -1*maxErr;

    rateError = (error-lastSpeedErrorR)/elapsedTime;      // Derivative Error

    double out = kpR*error+kiR*cumErrorR+kdR*rateError;   // PID output

    lastSpeedErrorR = error;                              // remember current error
    priorTimeR = currentTime;                             // remember current time
    return out;                                           // return the needed motor speed. 
}

int motorVelToSpeedCommand(double Vel, double maxVel){
    int newSpeed = 0;
    Vel = constrain(Vel,-1*maxVel, maxVel);
    newSpeed = map(Vel,-1*maxVel, maxVel, -400, 400);
    return newSpeed;
}
