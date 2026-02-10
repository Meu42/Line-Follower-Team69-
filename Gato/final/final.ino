#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>

// --- Pin Definitions ---
// Sensors
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
const uint8_t emitterPin = 4;

// Motor A (Left)
#define PIN_PWMA 19
#define PIN_AIN2 20
#define PIN_AIN1 21 // Your updated pin

// Motor B (Right)
#define PIN_PWMB 17
#define PIN_BIN1 15
#define PIN_BIN2 16

// System
#define PIN_STBY 3
#define RGB_PIN 48

QTRSensors qtr;
Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

// PID Tuning Constants (Adjust these to make it smoother!)
float Kp = 0.05; // Proportional: How hard to turn
float Kd = 0.25; // Derivative: Dampens the wobble
int lastError = 0;
const int baseSpeed = 160; // Cruise speed (0-255)

void setup() {
  Serial.begin(115200);
  pixels.begin();
  
  // Set Motor Pins
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  // Initialize Sensors
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);

  // Calibration Visual (Yellow)
  pixels.setPixelColor(0, pixels.Color(255, 150, 0)); pixels.show();
  Serial.println("Calibrating sensors... Move them over the line!");
  
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(10);
  }

  // Ready Visual (Green)
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); pixels.show();
  Serial.println("Calibration Done. Racing in 2 seconds!");
  delay(2000);
}

void setMotors(int left, int right) {
  // Constraints to keep speeds within 0-255
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);

  // Left Motor
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, left);

  // Right Motor
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMB, right);
}

void loop() {
  // readLineBlack returns 0-7000 (3500 is center)
  uint16_t position = qtr.readLineBlack(sensorValues);
  
  int error = (int)position - 3500;
  
  // PD Logic
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  // Differential Steering
  int leftMotorSpeed = baseSpeed + motorSpeed;
  int rightMotorSpeed = baseSpeed - motorSpeed;

  setMotors(leftMotorSpeed, rightMotorSpeed);
}