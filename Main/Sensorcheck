#include <QTRSensors.h>

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
const uint8_t emitterPin = 4;

QTRSensors qtr;

void setup() {
  Serial.begin(115200);
  
  // Initialize Sensors
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);

  delay(2000); // Give you time to open the Serial Monitor
  Serial.println("Starting Raw Sensor Check...");
  Serial.println("S0\tS1\tS2\tS3\tS4\tS5\tS6\tS7");
}

void loop() {
  // Read raw sensor values. 
  // For RC sensors, this returns a number between 0 and 2500.
  qtr.read(sensorValues);

  // Print all 8 values separated by a tab space
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); 
  }
  Serial.println(); 
  
  // Small delay so it doesn't flood the monitor too fast
  delay(150); 
}
