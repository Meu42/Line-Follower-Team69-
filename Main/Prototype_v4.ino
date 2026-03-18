//removed some functions that seemed to be limiting speed
//added BLE functionality to make it easier to update Kp and Kd values. Now the values can be updated by a simple command sent over BLE without connecting the robot to PC everytime.

#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

// --- Pin Definitions ---
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
Preferences preferences; // Object for Non-Volatile Memory storage

// --- Tuning Variables ---
float Kp = 0.04;
float Kd = 0.00;
int lastError = 0;
int baseSpeed = 180;

// --- State Machine Definitions ---
/**
 * ROBOT STATE MACHINE
 * WAITING_BLE: Initial boot, advertising Bluetooth signal.
 * IDLE: Connected to phone, but sensors need calibration.
 * CALIBRATING: Active reading of min/max sensor values.
 * READY: Calibrated and armed; waiting for the 'G' (Go) command.
 * RACING: Active PID control loop and motor movement.
 */
enum RobotState { WAITING_BLE, IDLE, CALIBRATING, READY, RACING };
volatile RobotState currentState = WAITING_BLE;
volatile bool isCalibrated = false;

// --- BLE Configuration ---
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

/**
 * HMI SIGNALING (LED COLORS)
 * Blue: Searching for a Bluetooth connection.
 * Purple: Connected to phone (Idle).
 * Yellow: Actively calibrating sensors.
 * Cyan (Light Blue): Ready to race.
 * Green: Racing mode active OR Save confirmation.
 * Red: Error (e.g., trying to race without calibrating).
 */
void setLED(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

// --- BLE Event Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      currentState = isCalibrated ? READY : IDLE;
      if (currentState == READY) setLED(0, 255, 255); // Cyan
      else setLED(128, 0, 128); // Purple
      Serial.println("Phone Connected!");
    };
    void onDisconnect(BLEServer* pServer) {
      currentState = WAITING_BLE;
      setLED(0, 0, 255); // Blue
      setMotors(0, 0);   // Safety Cutoff
      pServer->startAdvertising(); 
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue().c_str();

      if (rxValue.length() > 0) {
        char command = rxValue[0]; 
        float value = rxValue.substring(1).toFloat(); 

        // NAVIGATION COMMANDS
        if (command == 'C' || command == 'c') {
          currentState = CALIBRATING; // Trigger calibration loop
        }
        else if (command == 'G' || command == 'g') {
          if (isCalibrated) {
            currentState = RACING;
            setLED(0, 255, 0); // Green
          } else {
            setLED(255, 0, 0); // Red (Error)
            delay(500);
            setLED(128, 0, 128);
          }
        }
        else if (command == 'H' || command == 'h') {
          currentState = READY; // Halt robot instantly
          setLED(0, 255, 255); // Cyan
        }
        
        // TUNING COMMANDS (Direct Set)
        else if (command == 'P' || command == 'p') { Kp = value; } 
        else if (command == 'D' || command == 'd') { Kd = value; }
        else if (command == 'S' || command == 's') { baseSpeed = (int)value; }

        // TUNING COMMANDS (Real-time Incremental "Bumps")
        // 'I' & 'K' adjust Kp in small steps to observe stability while driving.
        else if (command == 'I' || command == 'i') { Kp += 0.005; }
        else if (command == 'K' || command == 'k') { Kp -= 0.005; if (Kp < 0) Kp = 0; }
        
        // 'O' & 'L' adjust Kd damping to stop fishtailing.
        else if (command == 'O' || command == 'o') { Kd += 0.0005; }
        else if (command == 'L' || command == 'l') { Kd -= 0.0005; if (Kd < 0) Kd = 0; }

        // PERMANENT STORAGE COMMAND
        // 'W' writes current variables to Flash so they survive a power cycle.
        else if (command == 'W' || command == 'w') {
          preferences.putFloat("Kp", Kp);
          preferences.putFloat("Kd", Kd);
          preferences.putInt("baseSpeed", baseSpeed);
          setLED(0, 255, 0); // Confirmation flash
          delay(500);
          currentState == READY ? setLED(0, 255, 255) : setLED(128, 0, 128);
        }
      }
    }
};

void setup() {
  Serial.begin(115200);

  // Load persistent tunings from Flash NVM
  preferences.begin("tuning", false); 
  Kp = preferences.getFloat("Kp", 0.05);
  Kd = preferences.getFloat("Kd", 0.25);
  baseSpeed = preferences.getInt("baseSpeed", 160);

  pixels.begin();
  setLED(0, 0, 255); // Start in Blue (Waiting for phone)
  
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);

  // Initialize BLE Stack
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
  BLEDevice::getAdvertising()->start();
}

void setMotors(int left, int right) {
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, left);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMB, right);
}

void loop() {
  switch (currentState) {
    case WAITING_BLE:
    case IDLE:
    case READY:
      setMotors(0, 0); // Safety: Motors must be OFF unless in RACING state.
      break;

    case CALIBRATING:
      setLED(255, 150, 0); // Yellow
      for (uint16_t i = 0; i < 200; i++) {
        qtr.calibrate();
        delay(10);
      }
      isCalibrated = true;
      currentState = READY;
      setLED(0, 255, 255); // Cyan
      break;

    case RACING:
      uint16_t position = qtr.readLineBlack(sensorValues);
      int error = (int)position - 3500;
      int motorSpeed = Kp * error + Kd * (error - lastError);
      lastError = error;

      int leftMotorSpeed = baseSpeed + motorSpeed;
      int rightMotorSpeed = baseSpeed - motorSpeed;
      setMotors(leftMotorSpeed, rightMotorSpeed);
      break;
  }
}

/* --------------------------------------------------------------------------------
Technical Details
--------------------------------------------------------------------------------

1. ASYNCHRONOUS EVENT-DRIVEN PROGRAMMING
Unlike standard Arduino code which is purely sequential, this robot utilizes 
the ESP32’s BLE stack to handle interrupts. When a phone sends a command, 
the `onWrite` callback executes immediately, potentially changing the 
`currentState` variable. The use of the `volatile` keyword ensures the CPU 
constantly checks the actual memory location of the state, preventing 
logic-stalling during high-speed racing.

--------------------------------------------------------------------------------

2. NON-VOLATILE MEMORY (NVM) PERSISTENCE
The `Preferences.h` library manages a dedicated flash partition. 
Traditional `EEPROM` is wear-prone and difficult to manage; `Preferences` 
uses a Key-Value storage system.
* Key "tuning": A namespace that protects the PID values from other apps.
* `putFloat` / `getFloat`: Automatically handles the 4-byte float conversion 
  to binary storage. This allows the robot to retain its "personality" and 
  tuning across power cycles.

--------------------------------------------------------------------------------

3. THE STATE MACHINE PATTERN
This robot implements a Finite State Machine (FSM). 

This is critical for safety in high-power robotics. By separating logic into 
mutually exclusive states (e.g., RACING vs READY), we ensure that the robot 
cannot physically move its motors unless the conditions (BLE Connected + 
Calibration Done + 'G' Command Received) are perfectly met.

--------------------------------------------------------------------------------

4. REAL-TIME PARAMETER BUMPING (HMI)
The incremental command set (I, K, O, L) creates a closed-loop tuning 
environment. In control engineering, this allows for "Heuristic Tuning," 
where a user can observe the robot's oscillation frequency and damp it 
digitally without stopping the physical test run.
--------------------------------------------------------------------------------
*/
