
const float WHEEL_DIAMETER_M = 0.040;    // 40 mm
const float GEAR_RATIO       = 1.0;      // 1:1
const float COUNTS_PER_REV_ENCODER = 48.0;
const float COUNTS_PER_REV_WHEEL   = COUNTS_PER_REV_ENCODER * GEAR_RATIO;
const float WHEEL_CIRCUMFERENCE_M  = 3.14159f * WHEEL_DIAMETER_M;

//////////////////// USER-SELECTABLE PWM ////////////////////
// Set these between 0 and 255 for each motor
const int PWM_A_CMD = 200;   // Motor A PWM command
const int PWM_B_CMD = 200;   // Motor B PWM command

//////////////////// PINS ////////////////////
// Motor A
const int ENA_PIN  = 9;
const int IN1_PIN  = 8;
const int IN2_PIN  = 7;
const int ENC_A_A  = 13; //PAY ATTENTION TO THIS
const int ENC_B_A  = 12;
// Motor B
const int ENB_PIN  = 4;
const int IN3_PIN  = 6;
const int IN4_PIN  = 5;
const int ENC_A_B  = 11; //PAY ATTENTION TO THIS
const int ENC_B_B  = 10;

//////////////////// ENCODER STATE ////////////////////
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

volatile uint8_t lastStateA = 0;
volatile uint8_t lastStateB = 0;

const int8_t quadTable[4][4] = {
  { 0, -1, +1, 0 },
  { +1, 0, 0, -1 },
  { -1, 0, 0, +1 },
  { 0, +1, -1, 0 }
};

unsigned long lastSample = 0;
long lastCountA = 0;
long lastCountB = 0;

void setMotorA(int pwm) {
  if (pwm >= 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    if (pwm > 255) pwm = 255;
    analogWrite(ENA_PIN, pwm);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    pwm = -pwm;
    if (pwm > 255) pwm = 255;
    analogWrite(ENA_PIN, pwm);
  }
}

void setMotorB(int pwm) {
  if (pwm >= 0) {
    // REVERSED FORWARD DIRECTION
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    if (pwm > 255) pwm = 255;
    analogWrite(ENB_PIN, pwm);
  } else {
    // REVERSED REVERSE DIRECTION
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    pwm = -pwm;
    if (pwm > 255) pwm = 255;
    analogWrite(ENB_PIN, pwm);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  pinMode(ENC_A_A, INPUT_PULLUP);
  pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP);
  pinMode(ENC_B_B, INPUT_PULLUP);

  // initial encoder state
  uint8_t pinb = PINB;
  lastStateA = (pinb >> 4) & 0x03;  // PB5, PB4
  lastStateB = (pinb >> 2) & 0x03;  // PB3, PB2

  // enable pin-change interrupts on PORTB
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT2) | (1 << PCINT3) | (1 << PCINT4) | (1 << PCINT5);

  // run motors forward at selected PWM
  setMotorA(PWM_A_CMD);
  setMotorB(PWM_B_CMD);

  lastSample = millis();
}

ISR(PCINT0_vect) {
  uint8_t current = PINB;

  // Motor A: PB5 (13) & PB4 (12)
  uint8_t stateA = (current >> 4) & 0x03;
  int8_t deltaA = quadTable[lastStateA][stateA];
  encoderCountA += deltaA;
  lastStateA = stateA;

  // Motor B: PB3 (11) & PB2 (10)
  uint8_t stateB = (current >> 2) & 0x03;
  int8_t deltaB = quadTable[lastStateB][stateB];
  encoderCountB += deltaB;
  lastStateB = stateB;
}

void loop() {
  unsigned long now = millis();
  if (now - lastSample >= 200) {  // every 200 ms
    float dt = (now - lastSample) / 1000.0f;
    lastSample = now;

    noInterrupts();
    long countA = encoderCountA;
    long countB = encoderCountB;
    interrupts();

    long dA = countA - lastCountA;
    long dB = countB - lastCountB;
    lastCountA = countA;
    lastCountB = countB;

    float speedA_mps = (dA / COUNTS_PER_REV_WHEEL) * WHEEL_CIRCUMFERENCE_M / dt;
    float speedB_mps = (dB / COUNTS_PER_REV_WHEEL) * WHEEL_CIRCUMFERENCE_M / dt;

    Serial.print("Motor A speed (m/s): ");
    Serial.print(speedA_mps, 3);
    Serial.print("   Motor B speed (m/s): ");
    Serial.println(speedB_mps, 3);
  }
}
