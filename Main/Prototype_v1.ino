//explanations are at the bottom
#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>

// --- Pin Definitions ---
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[] = {5, 6, 7, 8, 9, 10, 11, 12};
const uint8_t emitterPin = 4;

#define PIN_PWMA 19
#define PIN_AIN2 20
#define PIN_AIN1 21
#define PIN_PWMB 17
#define PIN_BIN1 15
#define PIN_BIN2 16
#define PIN_STBY 3
#define RGB_PIN 48

QTRSensors qtr;
Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

/**
 * PD TUNING PARAMETERS
 * Kp (Proportional): The "Strength" of the correction. High Kp makes the robot 
 * respond aggressively to off-center errors.
 * Kd (Derivative): The "Damper." It predicts the line's direction by looking at 
 * the rate of change, preventing the robot from overshooting (fishtailing).
 */
float Kp = 0.05;
float Kd = 0.25; 
int lastError = 0;
const int baseSpeed = 160; // The cruising speed when error is zero.

void setup() {
  Serial.begin(115200);
  pixels.begin();
  
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);

  // --- CALIBRATION TECHNIQUE ---
  // Sensors don't see "Black" or "White" inherently; they see infrared reflectance values
  // which vary based on floor material, ambient light, and battery voltage.
  // The loop below records the minimum (brightest) and maximum (darkest) values 
  // for each individual sensor to normalize them to a scale of 0-1000.
  pixels.setPixelColor(0, pixels.Color(255, 150, 0)); pixels.show();
  
  for (uint16_t i = 0; i < 200; i++) {
    // During this phase, the robot should be moved physically across the line
    // so every sensor "sees" both the full black and full white surfaces.
    qtr.calibrate();
    delay(10);
  }

  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); pixels.show();
  delay(2000);
}

/**
 * DIFFERENTIAL STEERING LOGIC
 * Unlike a car with a rack-and-pinion (steering wheel), this robot turns by 
 * varying the relative speeds of the two fixed wheels.
 * - Same speed = Straight line.
 * - Left slower/Right faster = Pivot Left.
 * - Left faster/Right slower = Pivot Right.
 */
void setMotors(int left, int right) {
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);

  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, left);

  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMB, right);
}

void loop() {
  // readLineBlack returns a weighted average (0 to 7000).
  // 3500 represents the line being perfectly centered under the 8 sensors.
  uint16_t position = qtr.readLineBlack(sensorValues);
  
  // Calculate Error: How far are we from the center (3500)?
  // Positive error = Line is to the right. Negative error = Line is to the left.
  int error = (int)position - 3500;
  
  // --- PD (PROPORTIONAL-DERIVATIVE) LOGIC ---
  // Proportional (Kp * error): The correction is proportional to the current distance.
  // Derivative (Kd * (error - lastError)): This calculates the slope (velocity) 
  // of the error. If the error is decreasing quickly, the D-term reduces the 
  // overall motor output to "brake" the turn and prevent overshooting the line.
  int motorSpeed = (Kp * error) + (Kd * (error - lastError));
  lastError = error;

  // --- APPLYING DIFFERENTIAL STEERING ---
  // We apply the motorSpeed as an offset to the baseSpeed.
  // If motorSpeed is positive (line is right), left motor speeds up and 
  // right motor slows down, causing a rightward turn.
  int leftMotorSpeed = baseSpeed + motorSpeed;
  int rightMotorSpeed = baseSpeed - motorSpeed;

  setMotors(leftMotorSpeed, rightMotorSpeed);
}

/* --------------------------------------------------------------------------------
TECHNICAL DETAILS
--------------------------------------------------------------------------------

1. SENSOR CALIBRATION AND DATA NORMALIZATION
IR sensors do not return a binary "black" or "white" value. Instead, they return 
a voltage corresponding to the amount of IR light reflected back.

THE NORMALIZATION TECHNIQUE:
The calibration process utilizes a Min-Max Scaling algorithm. During the 
calibration window, the robot records the extreme values for each sensor.
Each raw reading (S_raw) is then mapped to a standardized scale (0–1000) 
using the following logic:

    S_normalized = [ S_raw - S_min ] / [ S_max - S_min ] x 1000

This ensures that the `readLineBlack()` function always perceives the 
"darkest" point as 1000 and the "brightest" point as 0, regardless of 
ambient lighting or battery levels.

--------------------------------------------------------------------------------

2. CONTROL THEORY: PD LOGIC
To achieve smooth motion, the robot uses a Proportional-Derivative (PD) controller. 
This replaces simple "if-then" logic with a continuous mathematical function.

COMPONENT BREAKDOWN:
The control signal (u) is the sum of two distinct terms:

* PROPORTIONAL TERM (Kp x error): 
  The "restoring force." It generates a response proportional to the current 
  distance from center. Large error = Sharp turn; Small error = Gentle turn.

* DERIVATIVE TERM (Kd x de/dt) : 
  The "damping force." By calculating the difference between current error and 
  lastError, the robot determines its velocity toward the line. If the robot 
  returns to center too quickly, the D-term becomes negative, "braking" the 
  turn to prevent overshoot and oscillation.

THE RESULTING EQUATION:
    u(t) = Kp e(t) + Kd (e(t) - e(t-1))

--------------------------------------------------------------------------------

3. KINEMATICS: DIFFERENTIAL STEERING
This robot relies on Differential Drive kinematics, changing the angular velocity 
of two independent drive wheels to change the heading.

VELOCITY DISTRIBUTION:
The output of the PD controller (u) is applied as a differential offset to 
a constant baseSpeed (Vb):

| SCENARIO     | MATH                      | RESULT        |
|--------------|---------------------------|---------------|
| Centered     | u = 0                   | Straight      |
| Line Right   | u > 0                   | Pivot Right   |
| Line Left    | u < 0                   | Pivot Left    |

CONSTRAINTS AND SATURATION:
Motors have a physical PWM limit (0–255). The `constrain()` function prevents 
"Actuator Saturation," ensuring that the math doesn't command a speed the 
hardware cannot achieve, which would lead to unpredictable steering.
--------------------------------------------------------------------------------
*/
