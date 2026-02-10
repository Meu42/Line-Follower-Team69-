#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>

// --- Configuration & Pin Definitions ---
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
const uint8_t emitterPin = 4;

// ESP32-S3 N16R8 usually has the built-in RGB on GPIO 48
#define RGB_PIN 48 
Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

QTRSensors qtr;

// Timer variable for non-blocking serial output
unsigned long lastSerialTime = 0;
const int serialInterval = 100; // Print every 100ms

void setup() {
  // ESP32-S3 supports high baud rates; 115200 is standard
  Serial.begin(115200); 
  
  // Initialize NeoPixel for visual feedback
  pixels.begin();
  pixels.setBrightness(30);

  // Configure QTR Sensors
  qtr.setTypeRC(); // Use RC for digital timing-based sensors
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);

  // Visual Cue: Yellow = Calibrating
  pixels.setPixelColor(0, pixels.Color(255, 150, 0)); 
  pixels.show();

  Serial.println("--- ESP32-S3 Line Follower Initializing ---");
  Serial.println("Action: Move sensors over the line now...");

  // Optimized Calibration: Using a shorter loop with visual blinking
  for (uint16_t i = 0; i < 200; i++) {
    // Blink LED every 10 iterations to show activity
    if (i % 10 == 0) pixels.setPixelColor(0, (i % 20 == 0) ? pixels.Color(0,0,0) : pixels.Color(255, 150, 0));
    pixels.show();
    
    qtr.calibrate();
    delay(10); // Reduced delay for faster overall calibration
  }

  // Visual Cue: Green = Ready
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.show();
  
  Serial.println("Calibration Complete.");
  delay(1000); 
}

void loop() {
  // readLineBlack returns a value from 0 to (SensorCount-1)*1000
  // For 8 sensors, the center is 3500.
  uint16_t position = qtr.readLineBlack(sensorValues);

  // NON-BLOCKING SERIAL LOGGING
  // This ensures the robot doesn't "stutter" due to Serial print delays
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
    
    // Dynamic RGB Feedback: Blue if on line, Red if lost
    if (position == 0 || position == 7000) {
      pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red = Line Lost
    } else {
      pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // Blue = On Track
    }
    pixels.show();
  }

  // --- FUTURE PID LOGIC GOES HERE ---
  // int error = position - 3500; 
  // calculate_motor_speeds(error);
}