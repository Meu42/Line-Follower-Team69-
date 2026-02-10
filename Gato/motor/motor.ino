/* * ESP32-S3-N16R8 Motor Test for TB6612FNG
 * Pin mapping adjusted to avoid internal PSRAM conflicts (GPIO 35-37)
 */

#include <Adafruit_NeoPixel.h>

// --- Pin Definitions ---
// Motor A (Left)
#define PIN_PWMA 19
#define PIN_AIN2 20
#define PIN_AIN1 21

// Motor B (Right)
#define PIN_PWMB 17 // Shifted from 35 to avoid PSRAM conflict
#define PIN_BIN1 15 // Shifted from 37 to avoid PSRAM conflict
#define PIN_BIN2 16 // Shifted from 36 to avoid PSRAM conflict

// Control Pins
#define PIN_STBY 3  // Moved to GPIO 3 as requested
#define RGB_PIN 48  // Built-in RGB LED for ESP32-S3

Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  pixels.begin();
  pixels.setBrightness(30);

  // Initialize all motor driver pins as OUTPUT
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  // --- CRITICAL STEP ---
  // The TB6612FNG driver will remain in sleep mode until STBY is HIGH.
  digitalWrite(PIN_STBY, HIGH);
  
  Serial.println("System Ready. Motors testing in 3 seconds...");
  delay(3000);
}

void loop() {
  // Test Motor A (Left) - Green Light
  Serial.println("Testing Motor A (Forward)");
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); 
  pixels.show();
  
  digitalWrite(PIN_AIN1, HIGH); 
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, 150); // Speed range 0-255
  delay(1000);
  
  // Full Stop
  analogWrite(PIN_PWMA, 0);
  delay(500);

  // Test Motor B (Right) - Blue Light
  Serial.println("Testing Motor B (Forward)");
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); 
  pixels.show();
  
  digitalWrite(PIN_BIN1, HIGH); 
  digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMB, 150);
  delay(1000);

  // Full Stop
  analogWrite(PIN_PWMB, 0);
  delay(2000); 
}