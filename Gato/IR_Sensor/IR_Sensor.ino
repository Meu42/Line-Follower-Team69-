#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
const uint8_t emitterPin = 4;

#define RGB_PIN 48 
Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

QTRSensors qtr;

unsigned long lastSerialTime = 0;
const int serialInterval = 100; 

void setup() {
  Serial.begin(115200); 
  
  pixels.begin();
  pixels.setBrightness(30);

  //QTR setup
  qtr.setTypeRC(); 
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);

  // LED: Yellow --> Calibrating
  pixels.setPixelColor(0, pixels.Color(255, 150, 0)); 
  pixels.show();

  Serial.println("--- ESP32-S3 Line Follower Initializing ---");
  Serial.println("Action: Move sensors over the line now...");

  for (uint16_t i = 0; i < 200; i++) {
    // Blink LED every 10 iterations to show activity
    if (i % 10 == 0) pixels.setPixelColor(0, (i % 20 == 0) ? pixels.Color(0,0,0) : pixels.Color(255, 150, 0));
    pixels.show();
    
    qtr.calibrate();
    delay(10); 
  }

  // LED:Green --> Ready
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.show();
  
  Serial.println("Calibration Complete.");
  delay(1000); 
}

void loop() {
  // readLineBlack returns a value from 0 to (SensorCount-1)*1000
  // For 8 sensors, the center is 3500.
  uint16_t position = qtr.readLineBlack(sensorValues);

  if (millis() - lastSerialTime >= serialInterval) {
    lastSerialTime = millis();
    
    Serial.print("Pos: ");
    Serial.print(position);
    Serial.print(" | Sensors: ");
    for (uint8_t i = 0; i < SensorCount; i++) {
      Serial.print(sensorValues[i]);
      Serial.print(" ");
    }
    Serial.println();
    
    // Blue if on line, Red if lost
    if (position == 0 || position == 7000) {
      pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red = Line Lost
    } else {
      pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Blue = On Track
    }
    pixels.show();
  }
}
