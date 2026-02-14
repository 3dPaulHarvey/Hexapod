#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// Servo pulse range
#define SERVOMIN 150  // 0 degrees
#define SERVOMAX 650   // 180 degrees

#define POT_PIN A0

void setup() {
  Serial.begin(115200);

  Wire.begin();
  pca.begin();
  pca.setPWMFreq(50);

}

void loop() {
  int potValue = analogRead(POT_PIN);                 // 0–1023
  int pulse    = map(potValue, 0, 1023, SERVOMIN, SERVOMAX);
  int angle    = map(potValue, 0, 1023, 0, 180);

  pca.setPWM(0, 0, pulse);

  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" deg  |Blind|  Pulse: ");
  Serial.println(pulse);

  delay(100);   // Slow serial spam
}
