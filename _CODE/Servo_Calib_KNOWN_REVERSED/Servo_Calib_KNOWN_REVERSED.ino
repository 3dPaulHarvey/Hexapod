#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ===== USER HARDWARE =====
#define SERVO_CH 0
#define POT_PIN  A0

// ===== CALIBRATION VALUES (PER SERVO) =====
float centerPulse     = 297.0;     // true mechanical 90°
float pulsesPerDegree = 2.14;     // pulses per degree

// Safety limits (mechanical protection)
int pulseMin = 150;
int pulseMax = 650;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pca.begin();
  pca.setPWMFreq(50);

}

void loop() {
  int pot = analogRead(POT_PIN);          // 0–1023
  float angle = map(pot, 0, 1023, 0, 180);

  // ===== CORE CALIBRATION MATH ===== //reversed
  float pulse = centerPulse -
                (angle - 90.0) * pulsesPerDegree;

  // Safety clamp
  pulse = constrain(pulse, pulseMin, pulseMax);

  pca.setPWM(SERVO_CH, 0, (int)pulse);

  Serial.print("Angle: ");
  Serial.print(angle, 1);
  Serial.print(" deg |Known| Pulse: ");
  Serial.println(pulse, 1);

  delay(100);
}
