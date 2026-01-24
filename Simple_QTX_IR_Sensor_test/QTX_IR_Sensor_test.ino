#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Define your safe ESP32-S3 GPIOs here
const uint8_t sensorPins[] = {4, 5, 6, 7, 15, 16, 17, 18};
const uint8_t emitterPin = 1;

void setup() {
  Serial.begin(115200); // ESP32-S3 is faster, 115200 is standard
  
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);

  Serial.println("ESP32-S3: Reading raw QTR-8RC data...");
}

void loop() {
  qtr.read(sensorValues);

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(50); // Faster polling for the S3
}