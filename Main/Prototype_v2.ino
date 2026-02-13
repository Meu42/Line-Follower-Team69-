#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>

// --- Pin Definitions ---
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
const uint8_t emitterPin = 4;

// Motor Pins
#define PIN_PWMA 19
#define PIN_AIN2 20
#define PIN_AIN1 21
#define PIN_PWMB 17
#define PIN_BIN1 15
#define PIN_BIN2 16
#define PIN_STBY 3
#define RGB_PIN 48

QTRSensors qtr;
Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

// --- TUNING FOR SMOOTH TRACK ---
// Kp: Lower is better for smooth curves. 
// Too high, and it will oscillate on the long curves.
float Kp = 0.045; 
// Kd: increased to dampen the momentum of the robot at high speeds.
float Kd = 1.8; 

int lastError = 0;

// --- SPEED SETTINGS ---
// Zone 1 is fast, but Zone 2 (Roundabout) needs steady torque.
const int baseSpeed = 160; 
const int maxSpeed = 240;

void setup() {
  Serial.begin(115200);
  pixels.begin();
  pixels.setBrightness(30);

  // Motor Setup
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  // Sensor Setup
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);
  qtr.setSamplesPerSensor(6); // Slightly lower oversampling for faster response
  qtr.setTimeout(2500);

  // Calibration
  pixels.setPixelColor(0, pixels.Color(255, 150, 0)); pixels.show();
  Serial.println("Calibrating...");
  
  for (uint16_t i = 0; i < 300; i++) {
    qtr.calibrate();
    delay(10);
  }

  // Ready
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); pixels.show();
  Serial.println("Go!");
  delay(1000);
}

void setMotors(int left, int right) {
  // CONSTRAINT: 0 to maxSpeed. 
  // We DO NOT allow negative numbers (reverse) for smooth tracks.
  // This prevents the robot from jittering backward on a curve.
  left = constrain(left, 0, maxSpeed);
  right = constrain(right, 0, maxSpeed);

  // Motor A (Left)
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, left);

  // Motor B (Right)
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMB, right);
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  
  // --- INTERSECTION & FINISH LINE LOGIC ---
  int blackCount = 0;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 800) blackCount++;
  }

  // 1. FINISH LINE (Zone 1 Start/Finish)
  // If we see a wide black line (>6 sensors) AND we have been running for at least 10s
  // (This prevents stopping at the start line immediately)
  if (blackCount >= 7 && millis() > 10000) {
     setMotors(0, 0);
     pixels.setPixelColor(0, pixels.Color(255, 0, 0)); pixels.show();
     while(1); // Stop
  }

  // 2. INTERSECTION (Zone 3 Crossing)
  // If we see a cross (blackCount > 5) but we haven't been running long enough to finish,
  // it's likely the intersection. IGNORE IT by driving STRAIGHT.
  if (blackCount >= 5) {
    // Force straight drive for a split second to clear the intersection
    setMotors(baseSpeed, baseSpeed);
    delay(100); 
    return; // Skip the PID loop for this iteration
  }

  // --- PID CALCULATION ---
  int error = (int)position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int leftMotorSpeed = baseSpeed + motorSpeed;
  int rightMotorSpeed = baseSpeed - motorSpeed;

  setMotors(leftMotorSpeed, rightMotorSpeed);
}
