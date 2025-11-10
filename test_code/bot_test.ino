// Include the AccelStepper library
#include <AccelStepper.h>

// --- Stepper Pins (DRV8825) ---
#define DIR_PIN 2
#define STEP_PIN 3

// --- DC Motor Pins (L298N) ---
#define DC_MOTOR_IN1 7
#define DC_MOTOR_IN2 8

// Create an instance of the AccelStepper library
AccelStepper myStepper(1, STEP_PIN, DIR_PIN);

// Steps for 90 degrees with 1/16 microstepping
const int stepsFor90Degrees = 800;

// --- Timer variables for the stepper ---
unsigned long waitStartTime = 0;
// State variable: true if the stepper is waiting, false if it's moving
bool isStepperWaiting = true; 

void setup()
{
  Serial.begin(9600);
  while (!Serial); 
  Serial.println("--- Dual Motor Test Online ---");

  // --- DC Motor Setup ---
  pinMode(DC_MOTOR_IN1, OUTPUT);
  pinMode(DC_MOTOR_IN2, OUTPUT);
  Serial.println("DC Motor pins (8, 9) set to OUTPUT.");

  // Turn the DC motor ON (clockwise) to run forever
  Serial.println("-> DC Motor ON - Running full speed.");
  digitalWrite(DC_MOTOR_IN1, LOW);
  digitalWrite(DC_MOTOR_IN2, HIGH);

  // --- Stepper Motor Setup ---
  myStepper.setMaxSpeed(10000); 
  myStepper.setAcceleration(10000);
  Serial.println("-> Stepper (2, 3) settings applied.");
  Serial.println("----------------------------------------");


  // Start the stepper's first move immediately
  Serial.println("[Stepper ACTION] Rotating 90 degrees...");
  myStepper.move(stepsFor90Degrees);
  isStepperWaiting = false; // The stepper is now moving
}

void loop() 
{
  // --- This is the "engine" for the stepper ---
  // This MUST run every loop for AccelStepper to work.
  // It checks if a step is needed and then returns control.
  myStepper.run();

  // --- This is the "brain" for the stepper ---

  // Check if the stepper has just finished its move
  if (myStepper.distanceToGo() == 0 && !isStepperWaiting) {
    Serial.println("\n[Stepper EVENT] Stepper move complete.");
    
    // Start the 2-second wait
    Serial.println("  -> Starting 2-second wait...");
    waitStartTime = millis();
    isStepperWaiting = true;
  }

  // Check if the stepper is in its wait state
  if (isStepperWaiting) {
    // And check if the 2 seconds have passed
    if (millis() - waitStartTime >= 2000) {
      Serial.println("\n[Stepper EVENT] 2-second wait complete.");
      
      // Start the next stepper move
      Serial.println("[Stepper ACTION] Rotating 90 degrees...");
      myStepper.move(stepsFor90Degrees);
      
      isStepperWaiting = false; // The stepper is no longer waiting
    }
  }

  // --- DC Motor ---
  // No code is needed here. It was turned on in setup()
  // and will run forever.
}