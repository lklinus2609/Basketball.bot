// Include the AccelStepper library
#include <AccelStepper.h>

// --- Stepper Pins (DRV8825) ---
#define DIR_PIN 2
#define STEP_PIN 3

// Create an instance of the AccelStepper library
AccelStepper myStepper(1, STEP_PIN, DIR_PIN);

// Steps for 90 degrees with 1/16 microstepping
const int stepsFor90Degrees = 800;

// Timer variable for our 2-second wait
unsigned long waitStartTime = 0;
// State variable: true if we are waiting, false if we are moving
bool isWaiting = true; 

void setup()
{
  Serial.begin(9600);
  while (!Serial); 
  Serial.println("--- Stepper Motor Test Online ---");

  // Stepper settings
  myStepper.setMaxSpeed(10000); 
  myStepper.setAcceleration(10000);
  Serial.println("Stepper (2, 3) settings applied.");

  // Start the first move immediately
  Serial.println("[ACTION] Rotating 90 degrees...");
  myStepper.move(stepsFor90Degrees);
  isWaiting = false; // We are now moving
}

void loop() 
{
  // This MUST run every loop for AccelStepper to work
  myStepper.run();

  // Check if the stepper has just finished its move
  if (myStepper.distanceToGo() == 0 && !isWaiting) {
    Serial.println("\n[EVENT] Stepper move complete.");
    
    // Start the 2-second wait
    Serial.println("  -> Starting 2-second wait...");
    waitStartTime = millis();
    isWaiting = true;
  }

  // Check if we are in the wait state
  if (isWaiting) {
    // And check if the 2 seconds have passed
    if (millis() - waitStartTime >= 2000) {
      Serial.println("\n[EVENT] 2-second wait complete.");
      
      // Start the stepper move
      Serial.println("[ACTION] Rotating 90 degrees...");
      myStepper.move(stepsFor90Degrees);
      
      isWaiting = false; // We are no longer waiting
    }
  }
}