#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>

// ---------------- SENSOR SETUP ----------------

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

const uint8_t sensorPins[] = {5,6,7,8,9,10,11,12};
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

// ---------------- PID ----------------

float Kp = 0.065;
float Kd = 1.15;

int lastError = 0;

// ---------------- SPEED ----------------

const int MAX_SPEED = 255;
const int MIN_SPEED = 120;

unsigned long intersectionIgnoreUntil = 0;

// -------------------------------------------------

void setup()
{
  Serial.begin(115200);

  pixels.begin();
  pixels.setBrightness(30);

  // motor pins
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);

  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  // QTR setup
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(emitterPin);

  // more samples helps on reflective PVC
  qtr.setSamplesPerSensor(8);

  qtr.setTimeout(3000);

  // -------- calibration --------

  pixels.setPixelColor(0,pixels.Color(255,150,0));
  pixels.show();

  for(int i=0;i<400;i++)
  {
    qtr.calibrate();
    delay(8);
  }

  pixels.setPixelColor(0,pixels.Color(0,255,0));
  pixels.show();

  delay(1000);
}

// -------------------------------------------------

void setMotors(int left, int right)
{
  left = constrain(left,-255,255);
  right = constrain(right,-255,255);

  // LEFT MOTOR
  if(left >= 0)
  {
    digitalWrite(PIN_AIN1,HIGH);
    digitalWrite(PIN_AIN2,LOW);
  }
  else
  {
    digitalWrite(PIN_AIN1,LOW);
    digitalWrite(PIN_AIN2,HIGH);
    left = -left;
  }

  analogWrite(PIN_PWMA,left);

  // RIGHT MOTOR
  if(right >= 0)
  {
    digitalWrite(PIN_BIN1,HIGH);
    digitalWrite(PIN_BIN2,LOW);
  }
  else
  {
    digitalWrite(PIN_BIN1,LOW);
    digitalWrite(PIN_BIN2,HIGH);
    right = -right;
  }

  analogWrite(PIN_PWMB,right);
}

// -------------------------------------------------

void loop()
{
  uint16_t position = qtr.readLineBlack(sensorValues);

  int blackCount = 0;

  for(int i=0;i<SensorCount;i++)
  {
    if(sensorValues[i] > 500)
      blackCount++;
  }

  // -------- FINISH LINE --------

  if(blackCount >=7 && millis()>10000)
  {
    setMotors(0,0);

    pixels.setPixelColor(0,pixels.Color(255,0,0));
    pixels.show();

    while(1);
  }

  // -------- INTERSECTION --------

  if(blackCount >=5 && millis() > intersectionIgnoreUntil)
  {
    setMotors(180,180);

    intersectionIgnoreUntil = millis() + 120;

    return;
  }

  // -------- PID --------

  int error = position - 3500;

  int derivative = error - lastError;

  int correction = (Kp * error) + (Kd * derivative);

  lastError = error;

  correction = constrain(correction,-170,170);

  // -------- Adaptive Speed --------

  int speed = MAX_SPEED - abs(error)/6;

  speed = constrain(speed, MIN_SPEED, MAX_SPEED);

  // -------- Predictive Corner Detection --------

  int leftSide =
      sensorValues[0] +
      sensorValues[1] +
      sensorValues[2];

  int rightSide =
      sensorValues[5] +
      sensorValues[6] +
      sensorValues[7];

  if(leftSide > rightSide + 1200)
    speed -= 40;

  if(rightSide > leftSide + 1200)
    speed -= 40;

  // -------- Sharp Turn Protection --------

  if(abs(error) > 2500)
    speed = 130;

  int leftSpeed = speed + correction;
  int rightSpeed = speed - correction;

  // -------- Dynamic Braking --------

  if(abs(error) > 2600)
  {
    if(error > 0)
      rightSpeed = -80;
    else
      leftSpeed = -80;
  }

  setMotors(leftSpeed,rightSpeed);
}
