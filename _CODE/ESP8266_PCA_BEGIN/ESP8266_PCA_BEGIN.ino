#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo pulse range
#define SERVOMIN 300
#define SERVOMAX 400

// Servo channels
const uint8_t servoChannels[] = {7, 8, 9};
const uint8_t SERVO_COUNT = sizeof(servoChannels) / sizeof(servoChannels[0]);

void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println("Initializing I2C communication...");
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);

  Serial.println("I2C communication initialized.");
}

void loop() {
  Serial.println("Sweeping servos on channels 7, 8, 9...");

  // Sweep up
  for (int pulse = SERVOMIN; pulse <= SERVOMAX; pulse++) {
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      pwm.setPWM(servoChannels[i], 0, pulse);
    }
    delay(5);
  }

  // Sweep down
  for (int pulse = SERVOMAX; pulse >= SERVOMIN; pulse--) {
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
      pwm.setPWM(servoChannels[i], 0, pulse);
    }
    delay(5);
  }
}
