// --- TB6612 Motor Driver Pins ---
const int PWMA = 1; 
const int AIN1 = 2;
const int AIN2 = 4;
const int PWMB = 8;
const int BIN1 = 6;
const int BIN2 = 7;
const int STBY = 5;

// --- N20 Encoder Pins (Based on Green/Yellow feedback) ---
const int ENC_A_PHASE_A = 10; 
const int ENC_A_PHASE_B = 11;
const int ENC_B_PHASE_A = 12;
const int ENC_B_PHASE_B = 13;

// --- Constants (7 PPR for the encoder) ---
const float GEAR_RATIO = 30.0; // Update this to match your N20 (e.g., 10, 30, 50, 100)
const float TOTAL_PPR = GEAR_RATIO * 7.0; 

// --- Volatile Variables for Interrupts ---
volatile long posA = 0;
volatile long posB = 0;

// Tracking Variables
long lastPosA = 0;
long lastPosB = 0;
unsigned long lastMillis = 0;

void IRAM_ATTR readEncoderA() {
  if (digitalRead(ENC_A_PHASE_B) == HIGH) posA++;
  else posA--;
}

void IRAM_ATTR readEncoderB() {
  if (digitalRead(ENC_B_PHASE_B) == HIGH) posB++;
  else posB--;
}

void setup() {
  Serial.begin(115200);

  // Pin Modes
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(ENC_A_PHASE_A, INPUT_PULLUP);
  pinMode(ENC_A_PHASE_B, INPUT_PULLUP);
  pinMode(ENC_B_PHASE_A, INPUT_PULLUP);
  pinMode(ENC_B_PHASE_B, INPUT_PULLUP);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A_PHASE_A), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PHASE_A), readEncoderB, RISING);

  // PWM Setup
  ledcAttach(PWMA, 5000, 8);
  ledcAttach(PWMB, 5000, 8);

  digitalWrite(STBY, HIGH);
  Serial.println("LFR Motor Diagnostic Started...");
}

void loop() {
  // --- TEST MOTOR A ---
  Serial.println("\n--- Testing Motor A (Left) ---");
  runIndividualTest("A", 255); // Forward
  runIndividualTest("A", -255); // Backward
  stopAll();
  delay(1000);

  // --- TEST MOTOR B ---
  Serial.println("\n--- Testing Motor B (Right) ---");
  runIndividualTest("B", 180); // Forward
  runIndividualTest("B", -180); // Backward
  stopAll();
  delay(2000);
}

void runIndividualTest(String motor, int speed) {
  unsigned long start = millis();
  
  if (motor == "A") {
    if (speed > 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
    else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); }
    ledcWrite(PWMA, abs(speed));
  } else {
    if (speed > 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
    else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); }
    ledcWrite(PWMB, abs(speed));
  }

  while (millis() - start < 2000) {
    if (millis() - lastMillis >= 100) {
      float rpmA = ((posA - lastPosA) / TOTAL_PPR) * 600.0;
      float rpmB = ((posB - lastPosB) / TOTAL_PPR) * 600.0;

      if (motor == "A") {
        Serial.printf("MOTOR A | RPM: %.2f | Dir: %s\n", rpmA, (rpmA >= 0 ? "CW" : "CCW"));
      } else {
        Serial.printf("MOTOR B | RPM: %.2f | Dir: %s\n", rpmB, (rpmB >= 0 ? "CW" : "CCW"));
      }

      lastPosA = posA; lastPosB = posB;
      lastMillis = millis();
    }
  }
  stopAll();
}

void stopAll() {
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}
