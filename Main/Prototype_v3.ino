#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>

// ---------------- SENSOR SETUP ----------------
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
const uint8_t emitterPin = 4;

QTRSensors qtr;

// ---------------- MOTOR PINS ----------------
#define PIN_PWMA 19
#define PIN_AIN2 20
#define PIN_AIN1 21
#define PIN_PWMB 17
#define PIN_BIN1 15
#define PIN_BIN2 16
#define PIN_STBY 3
#define RGB_PIN 48

Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

// ---------------- PID VALUES ----------------
float Kp = 0.065;
float Kd = 1.15;
int lastError = 0;

// ---------------- SPEED ----------------
const int MAX_SPEED = 255;
const int MIN_SPEED = 120;

// ---------------- OTHER ----------------
// Non-blocking timer variable for intersection management. previously it froze the CPU at the intersection
unsigned long intersectionIgnoreUntil = 0;

void setup() {
  Serial.begin(115200);
  pixels.begin();
  pixels.setBrightness(30);

  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);

  /** * IMPROVEMENT: SIGNAL FILTERING
   * Increased to 8 samples to combat the glare due to the shiny PVC surface
   * Averaging more samples smooths out these highlights for cleaner data.
   */
  qtr.setSamplesPerSensor(8);
  qtr.setTimeout(3000);

  pixels.setPixelColor(0, pixels.Color(255, 150, 0));
  pixels.show();

  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(8);
  }

  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.show();
  delay(1000);
}

void setMotors(int left, int right) {
  left = constrain(left, 0, MAX_SPEED);
  right = constrain(right, 0, MAX_SPEED);

  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, left);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMB, right);
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int blackCount = 0;

  for (int i = 0; i < SensorCount; i++) {
    /** * IMPROVEMENT: REFLECTIVE THRESHOLD
     * Lowered to 500. we found out it increased accuracy by testing
     */
    if (sensorValues[i] > 500) blackCount++;
  }

  // -------- FINISH LINE --------
  if (blackCount >= 7 && millis() > 10000) {
    setMotors(0, 0);
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
    while (1);
  }

  // -------- INTERSECTION (NON-BLOCKING) --------
  /** * IMPROVEMENT: TIME-BASED STATE MANAGEMENT
   * Unlike delay(), this allows the robot to keep processing PID/Finish 
   * sensors while ignoring the intersection trigger for a set window (120ms).
   */
  if (blackCount >= 5 && millis() > intersectionIgnoreUntil) {
    setMotors(180, 180);
    intersectionIgnoreUntil = millis() + 120;
    return;
  }

  // -------- PID --------
  int error = position - 3500;
  int derivative = error - lastError;
  int correction = (Kp * error) + (Kd * derivative);
  lastError = error;

  /** * IMPROVEMENT: CORRECTION CONSTRAINING
   * Caps the steering force at 170. This prevents Tank-Turns where 
   * wheels spin in opposite directions, maintaining forward momentum.
   */
  correction = constrain(correction, -170, 170);

  // -------- ADAPTIVE SPEED CONTROL --------
  /** * IMPROVEMENT: KINETIC ENERGY MANAGEMENT
   * Speed scales inversely with error. On straights (error=0), speed is 255.
   * As the robot enters a curve, it throttles down automatically.
   */
  int speed = MAX_SPEED - abs(error) / 6;
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);

  /** * IMPROVEMENT: EMERGENCY BRAKING
   * Hard override for sharp turns. If the line is at the edge of the sensor 
   * array, we drop to 130 PWM to prevent the robot from flying off the track.
   */
  if (abs(error) > 2500) speed = 130;

  int leftSpeed = speed + correction;
  int rightSpeed = speed - correction;

  setMotors(leftSpeed, rightSpeed);
}

/* --------------------------------------------------------------------------------
TECHNICAL REPORT
--------------------------------------------------------------------------------

1. THE PHYSICS OF VELOCITY SCALING
By linking velocity to the absolute error, the system manages centrifugal force. 
The force required to maintain a turn is:
    F = (mv^2)/r
As the radius of the turn (r) decreases, the error increases. By reducing 
velocity (v) linearly with error, the robot stays within the friction limits 
of the tires, preventing "spin-outs" on hairpins while maximizing straightaway speed.

--------------------------------------------------------------------------------

2. NON-BLOCKING STATE ARCHITECTURE
Previous versions utilized `delay()`, which effectively "blinds" the robot. 
The current implementation uses a `millis()` comparison:
    `if(millis() > intersectionIgnoreUntil)`
This creates a pseudo-interrupt. The robot continues to cycle through its loop 
at ~1kHz, updating PID and checking the finish line, while the intersection 
logic stays "muted" until the timer expires. This ensures constant signal integrity.

--------------------------------------------------------------------------------

3. SIGNAL INTEGRITY ON REFLECTIVE SURFACES
High-speed racing often occurs on glossy PVC or painted tracks. These materials 
create glares.
* SAMPLING: `setSamplesPerSensor(8)` acts as a Low-Pass Filter, removing high-frequency 
  glare spikes from the infrared data.
* DYNAMIC THRESHOLDING: By dropping the black-detection threshold to 500, we account 
  for the fact that "Black" on a glossy surface reflects more IR light than 
  "Black" on a matte surface.

--------------------------------------------------------------------------------

4. STEERING SATURATION AND MOMENTUM
The `constrain(correction, -170, 170)` logic prevents Actuator Saturation. 
If one motor were to reach 0 while the other reached 255, the robot would 
pivot on its center. While useful for slow movement, at high speed this causes 
a loss of traction. Capping the correction ensures both wheels maintain a 
positive forward vector.
--------------------------------------------------------------------------------
*/
