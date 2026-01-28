// --- TB6612 Motor Driver Pins ---
const int PWMA = 1; 
const int AIN1 = 2;
const int AIN2 = 4;
const int PWMB = 8;
const int BIN1 = 6;
const int BIN2 = 7;
const int STBY = 5;

void setup() {
  // Pin Modes
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // PWM Setup (ESP32 ledc style)
  ledcAttach(PWMA, 5000, 8);
  ledcAttach(PWMB, 5000, 8);

  // Enable the motor driver
  digitalWrite(STBY, HIGH);
}

void loop() {
  // --- Move Forward ---
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); ledcWrite(PWMA, 255);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); ledcWrite(PWMB, 255);
  delay(1000); // Adjust this for how long you want it to move forward

  // --- Wait ---
  stopAll();
  delay(500);

  // --- Move Backward ---
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); ledcWrite(PWMA, 255);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); ledcWrite(PWMB, 255);
  delay(1000); // Adjust this for how long you want it to move backward

  // --- Wait ---
  stopAll();
  delay(500);
}

void stopAll() {
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}
