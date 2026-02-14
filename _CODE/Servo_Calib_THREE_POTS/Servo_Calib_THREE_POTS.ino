#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

#define SERVO_MIN 150
#define SERVO_MAX 600

#define POT0 A0   // controls servo channel 0
#define POT1 A1   // controls servo channel 1
#define POT2 A2   // controls servo channel 2

// Permanent direction settings
const bool CH0_REVERSED = false;
const bool CH1_REVERSED = true;   // reversed
const bool CH2_REVERSED = true;   // reversed

uint16_t angleToPulse(float angle, bool reversed) {
  if (reversed) angle = 180.0 - angle;
  angle = constrain(angle, 0.0, 180.0);
  return map((int)angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);
  delay(10);
}

void loop() {
  // Read pots
  float angle0 = map(analogRead(POT0), 0, 1023, 0, 180);
  float angle1 = map(analogRead(POT1), 0, 1023, 0, 180);
  float angle2 = map(analogRead(POT2), 0, 1023, 0, 180);

  // Convert to pulses
  uint16_t pulse0 = angleToPulse(angle0, CH0_REVERSED);
  uint16_t pulse1 = angleToPulse(angle1, CH1_REVERSED);
  uint16_t pulse2 = angleToPulse(angle2, CH2_REVERSED);

  // Drive servos
  pca.setPWM(0, 0, pulse0);
  pca.setPWM(1, 0, pulse1);
  pca.setPWM(2, 0, pulse2);

  // Debug output
  Serial.print("CH0: ");
  Serial.print(angle0);
  Serial.print("° | ");
  Serial.print(pulse0);

  Serial.print("   CH1: ");
  Serial.print(angle1);
  Serial.print("° | ");
  Serial.print(pulse1);

  Serial.print("   CH2: ");
  Serial.print(angle2);
  Serial.print("° | ");
  Serial.println(pulse2);

  delay(50);
}
