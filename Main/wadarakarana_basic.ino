#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

// --- Pin Definitions (From Original Code) ---
// Sensors
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
const uint8_t emitterPin = 4;

// Motor A (Left)
#define PIN_PWMA 19
#define PIN_AIN2 21
#define PIN_AIN1 20

// Motor B (Right)
#define PIN_PWMB 17
#define PIN_BIN1 15
#define PIN_BIN2 16

// System
#define PIN_STBY 3
#define RGB_PIN 48

QTRSensors qtr;
Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
Preferences preferences;

// --- Tuning Constants (From Original Code) ---
float Kp = 0.04;
float Kd = 0.00;
int lastError = 0;
int baseSpeed = 180;

// --- Forward Declarations ---
void setMotors(int left, int right);
void setLED(uint8_t r, uint8_t g, uint8_t b);

// --- BLE State Machine ---
enum RobotState { WAITING_BLE, IDLE, CALIBRATING, READY, RACING };
volatile RobotState currentState = WAITING_BLE;
volatile bool isCalibrated = false;

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

void setLED(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

// --- BLE Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      currentState = isCalibrated ? READY : IDLE;
      if (currentState == READY) {
        setLED(0, 255, 255); // Cyan: Ready
      } else {
        setLED(128, 0, 128); // Purple: Idle/Connected
      }
      Serial.println("Phone Connected!");
    };
    void onDisconnect(BLEServer* pServer) {
      currentState = WAITING_BLE;
      setLED(0, 0, 255); // Blue: Waiting
      setMotors(0, 0);   // Safety stop
      Serial.println("Phone Disconnected. Restarting advertising...");
      pServer->startAdvertising(); 
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue().c_str();

      if (rxValue.length() > 0) {
        char command = rxValue[0]; 
        float value = rxValue.substring(1).toFloat(); 

        if (command == 'C' || command == 'c') {
          currentState = CALIBRATING;
          Serial.println("Command: Start Calibration");
        }
        else if (command == 'G' || command == 'g') {
          if (isCalibrated) {
            currentState = RACING;
            setLED(0, 255, 0); // Green: Racing
            Serial.println("Command: GO!");
          } else {
            setLED(255, 0, 0); // Red: Error, not calibrated
            Serial.println("Error: Must calibrate first!");
            delay(500);
            setLED(128, 0, 128); // Back to purple
          }
        }
        else if (command == 'H' || command == 'h') {
          currentState = READY;
          setLED(0, 255, 255); // Cyan: Ready/Halted
          Serial.println("Command: HALT");
        }
        
        // --- Tuning Direct Setting Commands ---
        else if (command == 'P' || command == 'p') {
          Kp = value;
          Serial.print("Kp set to: "); Serial.println(Kp, 4);
        } 
        else if (command == 'D' || command == 'd') {
          Kd = value;
          Serial.print("Kd set to: "); Serial.println(Kd, 4);
        }
        else if (command == 'S' || command == 's') {
          baseSpeed = (int)value;
          Serial.print("Base Speed set to: "); Serial.println(baseSpeed);
        }

        // --- Incremental Bump Commands ---
        else if (command == 'I' || command == 'i') {
          Kp += 0.005; Serial.print("Kp bumped UP to: "); Serial.println(Kp, 4);
        }
        else if (command == 'K' || command == 'k') {
          Kp -= 0.005; if (Kp < 0.0) Kp = 0.0; Serial.print("Kp bumped DOWN to: "); Serial.println(Kp, 4);
        }
        else if (command == 'O' || command == 'o') {
          Kd += 0.0005; Serial.print("Kd bumped UP to: "); Serial.println(Kd, 4);
        }
        else if (command == 'L' || command == 'l') {
          Kd -= 0.0005; if (Kd < 0.0) Kd = 0.0; Serial.print("Kd bumped DOWN to: "); Serial.println(Kd, 4);
        }

        // --- Permanent Save Command ---
        else if (command == 'W' || command == 'w') {
          preferences.putFloat("Kp", Kp);
          preferences.putFloat("Kd", Kd);
          preferences.putInt("baseSpeed", baseSpeed);
          
          setLED(0, 255, 0); // Flash Green to confirm save
          Serial.println("SUCCESS: Tunings permanently saved to flash memory!");
          delay(500);
          
          if (currentState == READY) setLED(0, 255, 255); 
          else setLED(128, 0, 128); 
        }
      }
    }
};

void setup() {
  Serial.begin(115200);

  // Load Tunings from Memory (using your original code's defaults as fallback)
  preferences.begin("tuning", false); 
  Kp = preferences.getFloat("Kp", 0.05);
  Kd = preferences.getFloat("Kd", 0.25);
  baseSpeed = preferences.getInt("baseSpeed", 160);

  pixels.begin();
  setLED(0, 0, 255); // Blue: WAITING_BLE
  
  // Set Motor Pins
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  // Initialize Sensors
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);

  // BLE Setup
  BLEDevice::init("S3_RaceBot"); 
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_WRITE  |
                        BLECharacteristic::PROPERTY_NOTIFY |
                        BLECharacteristic::PROPERTY_INDICATE
                      );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  
  BLEDevice::startAdvertising();
  Serial.println("BLE Ready! Waiting for connection...");
}

void setMotors(int left, int right) {
  // Constraints to keep speeds within 0-255 (From Original Code)
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
  switch (currentState) {
    case WAITING_BLE:
    case IDLE:
    case READY:
      setMotors(0, 0); 
      break;

    case CALIBRATING:
      setLED(255, 150, 0); 
      Serial.println("Calibrating sensors... Move them over the line!");
      for (uint16_t i = 0; i < 200; i++) {
        qtr.calibrate();
        delay(10);
      }
      isCalibrated = true;
      currentState = READY;
      setLED(0, 255, 255); 
      Serial.println("Calibration Done. Send 'G' to GO.");
      break;

    case RACING:
      // Original Code PD Logic
      uint16_t position = qtr.readLineBlack(sensorValues);
      
      int error = (int)position - 3500;
      
      int motorSpeed = Kp * error + Kd * (error - lastError);
      lastError = error;

      // Differential Steering
      int leftMotorSpeed = baseSpeed + motorSpeed;
      int rightMotorSpeed = baseSpeed - motorSpeed;

      setMotors(leftMotorSpeed, rightMotorSpeed);
      break;
  }
}
