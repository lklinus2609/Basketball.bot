// --- DC Motor Pins (L298N) ---
#define DC_MOTOR_IN1 7
#define DC_MOTOR_IN2 8

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("--- DC Motor Test Online ---");

  // Set up the DC motor pins as outputs
  pinMode(DC_MOTOR_IN1, OUTPUT);
  pinMode(DC_MOTOR_IN2, OUTPUT);
  Serial.println("DC Motor pins (8, 9) set to OUTPUT.");
}

void loop() {
  // Go clockwise for 1 second
  Serial.println("Motor ON - Clockwise");
  digitalWrite(DC_MOTOR_IN1, HIGH);
  digitalWrite(DC_MOTOR_IN2, LOW);


}