#include <QTRSensors.h>

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
const uint8_t emitterPin = 4;

QTRSensors qtr;

void setup() {
  Serial.begin(115200);
  
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);

  delay(2000); 
  Serial.println("Starting Raw Sensor Check...");
  Serial.println("S0\tS1\tS2\tS3\tS4\tS5\tS6\tS7");
}

void loop() {
  qtr.read(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); 
  }
  Serial.println(); 
  delay(150); 
}
