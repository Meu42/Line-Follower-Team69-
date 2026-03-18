#include <Adafruit_NeoPixel.h>
#include <QTRSensors.h>
#include <NimBLEDevice.h>

// Hardware Pins
#define LED_PIN    48 
#define NUMPIXELS  1 
#define B_PIN      15
#define SENSOR_COUNT 8

// BLE UUIDs (You can generate your own, but these work for testing)
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Objects
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

// BLE Globals
BLECharacteristic* pCharacteristic;
bool deviceConnected = false;

// Setup sensor pins (Adjust these GPIOs to your actual wiring!)
const uint8_t sensorPins[SENSOR_COUNT] = {5,6,7,8,9,10,11,12};

void setup() {
  Serial.begin(115200);
  
  // NeoPixel & Buzzer Init
  pixels.begin(); 
  pixels.setBrightness(100);
  pinMode(B_PIN, OUTPUT);
  startup();

  // 1. QTR Sensor Setup
  qtr.setTypeRC(); // Use setTypeAnalog() if using the analog version
  qtr.setSensorPins(sensorPins, SENSOR_COUNT);

  // 2. BLE Setup
  BLEDevice::init("ESP32_QTR_Sensor");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
                    );
  pService->start();
  BLEDevice::startAdvertising();
  
  Serial.println("BLE Advertising Started...");
}

void loop() {
  // Read sensor data (values 0 - 2500)
  qtr.read(sensorValues);

  // Prepare string to send over BLE
  String dataString = "";
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    dataString += String(sensorValues[i]);
    if (i < SENSOR_COUNT - 1) dataString += ",";
  }

  // Update BLE characteristic
  pCharacteristic->setValue(dataString.c_str());
  pCharacteristic->notify();
  Serial.println(dataString.c_str());

  // Visual feedback on the LED if the first sensor sees "black"
  if(sensorValues[0] > 1000) {
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  } else {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  }
  pixels.show();

  delay(100); 
}

void startup() {
  communication(100, 3, 0, 255, 255); // Cyan flash on boot
}

void communication(int t, int r, int R, int G, int B) {
  for(int i=0; i<r; i++){
    pixels.clear();
    digitalWrite(B_PIN, HIGH);
    pixels.setPixelColor(0, pixels.Color(R, G, B));
    pixels.show();
    delay(t);
    digitalWrite(B_PIN, LOW);
    pixels.clear();
    pixels.show();
    delay(t);
  }
}
