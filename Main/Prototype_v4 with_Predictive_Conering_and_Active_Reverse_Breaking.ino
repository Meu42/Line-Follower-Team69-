#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

// --- Hardware Configuration ---
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
const uint8_t emitterPin = 4;

#define PIN_PWMA 19
#define PIN_AIN2 21
#define PIN_AIN1 20
#define PIN_PWMB 17
#define PIN_BIN1 15
#define PIN_BIN2 16
#define PIN_STBY 3
#define RGB_PIN 48

QTRSensors qtr;
Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
Preferences preferences;

// --- Control Constants & Tuning ---
float Kp = 0.04;
float Kd = 0.20;
float corneringGain = 0.03; //The "Brake Sensitivity" for curves.
int lastError = 0;
int baseSpeed = 180;
int minCornerSpeed = 80;   //Prevents the robot from stopping entirely in a turn.

// --- BLE State Machine ---
/**
 * HMI SIGNALING (LED COLORS)
 * Blue: Waiting for BLE connection.
 * Purple: Connected but needs calibration.
 * Cyan: Calibrated and ready to race.
 * Yellow: Actively calibrating.
 * Green: Racing mode active.
 */
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

/**
 * IMPROVEMENT: ACTIVE REVERSE BRAKING
 * The previous setMotors only handled 0-255. This version allows negative 
 * values (down to -255), enabling the robot to reverse one motor to 
 * perform a faster turn" 
 */
void setMotors(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // Left Motor Phase Logic
  if (left >= 0) {
    digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
  } else {
    digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, HIGH); // Reverse
  }
  analogWrite(PIN_PWMA, abs(left));

  // Right Motor Phase Logic
  if (right >= 0) {
    digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
  } else {
    digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, HIGH); // Reverse
  }
  analogWrite(PIN_PWMB, abs(right));
}

// --- BLE Callbacks (Command Set) ---
/**
 * COMMAND DICTIONARY:
 * 'C' - Calibrate Sensors.
 * 'G' - Go (Start Racing).
 * 'H' - Halt (Emergency Stop).
 * 'P','D','S','T' - Set Kp, Kd, Speed, and Cornering Gain.
 * 'W' - Write to Flash (Save).
 */
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      currentState = isCalibrated ? READY : IDLE;
      setLED(isCalibrated ? 0 : 128, 255, isCalibrated ? 255 : 128);
    };
    void onDisconnect(BLEServer* pServer) {
      currentState = WAITING_BLE;
      setLED(0, 0, 255);
      setMotors(0, 0);
      pServer->startAdvertising(); 
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue().c_str();
      if (rxValue.length() > 0) {
        char command = rxValue[0]; 
        float value = rxValue.substring(1).toFloat(); 

        if (command == 'C' || command == 'c') currentState = CALIBRATING;
        else if (command == 'G' || command == 'g') {
          if (isCalibrated) { currentState = RACING; setLED(0, 255, 0); }
        }
        else if (command == 'H' || command == 'h') {
          currentState = READY; setLED(0, 255, 255);
        }
        else if (command == 'P') Kp = value;
        else if (command == 'D') Kd = value;
        else if (command == 'S') baseSpeed = (int)value;
        else if (command == 'T') corneringGain = value; 

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

  // Load saved tunings from NVS (Non-Volatile Storage)
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

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);

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
      uint16_t position = qtr.readLineBlack(sensorValues);
      int error = (int)position - 3500;

      /** * IMPROVEMENT: PREDICTIVE CORNERING LOGIC
       * Instead of a static baseSpeed, we scale speed based on the error.
       * Large Error = Sharp Turn = Lower Speed.
       * Small Error = Straight = High Speed.
       */
      int dynamicBaseSpeed = baseSpeed - (abs(error) * corneringGain);
      if (dynamicBaseSpeed < minCornerSpeed) dynamicBaseSpeed = minCornerSpeed;

      // PD Steering Correction
      int motorCorrection = (Kp * error) + (Kd * (error - lastError));
      lastError = error;

      int leftMotorSpeed = dynamicBaseSpeed + motorCorrection;
      int rightMotorSpeed = dynamicBaseSpeed - motorCorrection;

      setMotors(leftMotorSpeed, rightMotorSpeed);
      break;
  }
}

/* --------------------------------------------------------------------------------
TECHNICAL DETAILS
--------------------------------------------------------------------------------

1. PREDICTIVE CORNERING (VARIABLE VELOCITY)
The core achievement of this code is the "Dynamic Base Speed" formula:
    V_{target} = V_{base} - (|error| x G_{corner})
This mimics a human driver's behavior. By decreasing the forward velocity (V) 
proportionally to the steering error, the robot maintains maximum grip 
(traction) during high-G turns while maximizing acceleration on straights. 
This significantly reduces the lap time compared to a fixed-speed robot.



--------------------------------------------------------------------------------

2. ACTIVE REVERSE BRAKING (4-QUADRANT CONTROL)
Standard line followers only "slow down" a wheel to turn. By utilizing the 
full H-Bridge logic (AIN1/AIN2 phase switching), this code allows for 
negative PWM values. 
* If `motorCorrection` > `dynamicBaseSpeed`, the inner wheel will actively 
  spin BACKWARDS.

--------------------------------------------------------------------------------

3. PERSISTENT STATE MACHINE (NVS)
Using the ESP32 `Preferences` library, the robot treats its tuning parameters 
as a "Profile." 
* NVS partitions are more reliable than standard EEPROM. 
* By storing `corneringGain`, the robot remembers how "cautious" it should 
  be on turns even after a battery swap, making it a reliable competition tool.

--------------------------------------------------------------------------------
*/
