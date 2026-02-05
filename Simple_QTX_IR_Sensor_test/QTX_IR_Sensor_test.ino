#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
  // Configure the sensor type as RC
  qtr.setTypeRC();
  
  // Define the pins used for the 8 sensors
  qtr.setSensorPins((const uint8_t[]){5,6,7,8,9,10,11,12}, SensorCount);
  
  // Set the emitter control pin (optional)
  qtr.setEmitterPin(4);

  Serial.begin(9600);
  Serial.println("Reading raw QTR-8RC data (No calibration)...");
}

void loop() {
  // read() gives raw values in microseconds (0 to 2500 by default)
  // Higher values = Darker surface
  qtr.read(sensorValues);

  // Print values to Serial Monitor
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(100); 
}
