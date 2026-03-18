#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

/* * --- HARDWARE CONFIGURATION ---
 * We define the pins for the QTR sensors, TB6612FNG motor driver, and the onboard RGB.
 * The 'const' keyword ensures these values aren't accidentally changed during runtime.
 */
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
const uint8_t emitterPin = 4;

// Motor A (Left) - Using PWM for speed and two pins for phase/direction
#define PIN_PWMA 19
#define PIN_AIN2 21
#define PIN_AIN1 20

// Motor B (Right)
#define PIN_PWMB 17
#define PIN_BIN1 15
#define PIN_BIN2 16

// System Pins
#define PIN_STBY 3    // Standby pin for motor driver (Must be HIGH to run)
#define RGB_PIN 48    // Built-in WS2812 LED on most S3 boards

/* * --- OBJECT INITIALIZATION ---
 * QTRSensors: Handles the IR reflectance array math.
 * Preferences: Used for non-volatile storage (NVS) to keep tunings after power-off.
 */
QTRSensors qtr;
Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
Preferences preferences;

/* * --- CONTROL CONSTANTS & TUNING ---
 * These variables dictate the "personality" of the robot's movement.
 */
float Kp = 0.04;           // Proportional: Correction based on current distance from center.
float Kd = 0.20;           // Derivative: Dampens oscillation by reacting to change rate.
float corneringGain = 0.03; // Predictive: How much to "brake" when the error increases.
int lastError = 0;         // Stores the previous error to calculate the Derivative (Slope).
int baseSpeed = 180;       // Straight-line target speed (0-255).
int minCornerSpeed = 80;   // The floor for dynamic speed scaling to avoid motor stall.

// --- BLE STATE MACHINE ---
// Volatile ensures the compiler doesn't "optimize away" variables changed in interrupts.
enum RobotState { WAITING_BLE, IDLE, CALIBRATING, READY, RACING };
volatile RobotState currentState = WAITING_BLE;
volatile bool isCalibrated = false;

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// --- HELPER FUNCTIONS ---

void setLED(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

/**
 * TECHNICAL LOGIC: Differential Drive
 * We take a left/right speed and map it to the TB6612FNG logic.
 * Forward: AIN1=HIGH, AIN2=LOW. Backward: AIN1=LOW, AIN2=HIGH.
 */
void setMotors(int left, int right) {
  left = constrain(left, -255, 255);   // Allow negative values for active reverse braking
  right = constrain(right, -255, 255);

  // Left Motor Logic
  if (left >= 0) {
    digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
  } else {
    digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, HIGH);
  }
  analogWrite(PIN_PWMA, abs(left));

  // Right Motor Logic
  if (right >= 0) {
    digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
  } else {
    digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, HIGH);
  }
  analogWrite(PIN_PWMB, abs(right));
}

// --- BLE CALLBACKS ---

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      currentState = isCalibrated ? READY : IDLE;
      setLED(isCalibrated ? 0 : 128, 255, isCalibrated ? 255 : 128);
      Serial.println("Phone Connected!");
    };
    void onDisconnect(BLEServer* pServer) {
      currentState = WAITING_BLE;
      setLED(0, 0, 255);
      setMotors(0, 0);
      Serial.println("Phone Disconnected. Advertising...");
      pServer->startAdvertising(); 
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue().c_str();
      if (rxValue.length() > 0) {
        char command = rxValue[0]; 
        float value = rxValue.substring(1).toFloat(); 

        // Handle Movement Commands
        if (command == 'C' || command == 'c') currentState = CALIBRATING;
        else if (command == 'G' || command == 'g') {
          if (isCalibrated) { currentState = RACING; setLED(0, 255, 0); }
        }
        else if (command == 'H' || command == 'h') {
          currentState = READY; setLED(0, 255, 255);
        }
        
        // Handle Tuning Updates
        else if (command == 'P') Kp = value;
        else if (command == 'D') Kd = value;
        else if (command == 'S') baseSpeed = (int)value;
        else if (command == 'T') corneringGain = value; // New BLE command for Cornering

        // Permanent Save
        else if (command == 'W') {
          preferences.putFloat("Kp", Kp);
          preferences.putFloat("Kd", Kd);
          preferences.putFloat("C_Gain", corneringGain);
          preferences.putInt("baseSpeed", baseSpeed);
          Serial.println("Tunings Saved!");
        }
      }
    }
};

void setup() {
  Serial.begin(115200);

  /* * Preferences.begin("tuning", false) opens a "namespace" in Flash memory.
   * The 'false' parameter means Read/Write mode.
   */
  preferences.begin("tuning", false); 
  Kp = preferences.getFloat("Kp", 0.04);
  Kd = preferences.getFloat("Kd", 0.20);
  corneringGain = preferences.getFloat("C_Gain", 0.03);
  baseSpeed = preferences.getInt("baseSpeed", 180);

  pixels.begin();
  setLED(0, 0, 255); 
  
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  /*
   * Initialize QTR as RC (Resistance-Capacitance) type.
   * This calibrates the 'dark' vs 'light' thresholds for your specific track.
   */
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);

  // BLE initialization (Omitted details for brevity, remains as original)
  BLEDevice::init("S3_RaceBot"); 
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, 
                    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | 
                    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEDevice::startAdvertising();
}

void loop() {
  switch (currentState) {
    case WAITING_BLE:
    case IDLE:
    case READY:
      setMotors(0, 0); 
      break;

    case CALIBRATING:
      /* * TECHNICAL LOGIC: Calibration
       * We run 200 samples. During this, you must physically move the robot 
       * over the line so it sees the minimum (black) and maximum (white) values.
       */
      setLED(255, 150, 0); 
      for (uint16_t i = 0; i < 200; i++) {
        qtr.calibrate();
        delay(10);
      }
      isCalibrated = true;
      currentState = READY;
      setLED(0, 255, 255); 
      break;

    case RACING:
      /* * 1. SENSOR FUSION & POSITIONING
       * readLineBlack returns a weighted average (0 to 7000). 
       * 3500 is the mathematical center of the 8-sensor array.
       */
      uint16_t position = qtr.readLineBlack(sensorValues);
      int error = (int)position - 3500;

      /* * 2. PREDICTIVE CORNERING (THE "BRAKE" LOGIC)
       * We calculate a dynamic base speed. The larger the error (sharp turn),
       * the more we subtract from the straight-line speed.
       * Math: $v_{target} = v_{base} - (|error| \times G_{corner})$
       */
      int dynamicBaseSpeed = baseSpeed - (abs(error) * corneringGain);
      if (dynamicBaseSpeed < minCornerSpeed) dynamicBaseSpeed = minCornerSpeed;

      /* * 3. PD CONTROL CALCULATION
       * Proportional ($Kp \times error$): Instant reaction to position.
       * Derivative ($Kd \times (error - lastError)$): Prediction based on error velocity.
       * If error is increasing fast, Kd provides a counter-acting force.
       */
      int motorCorrection = (Kp * error) + (Kd * (error - lastError));
      lastError = error;

      /* * 4. DIFFERENTIAL OUTPUT
       * We combine the slowed-down base speed with the steering correction.
       */
      int leftMotorSpeed = dynamicBaseSpeed + motorCorrection;
      int rightMotorSpeed = dynamicBaseSpeed - motorCorrection;

      setMotors(leftMotorSpeed, rightMotorSpeed);
      break;
  }
}
