// Converted for ESP8266 and Serial Communication
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"
#include "motion.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

MotionMode currentMotion = Mode_Standby;

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz
  Serial.println("Hexapod ready. Use serial commands to control.");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    handleCommand(command);
  }
}

void handleCommand(String command) {
  if (command == "standby") {
    currentMotion = Mode_Standby;
    executeMotion(currentMotion);
  } else if (command == "walk_forward") {
    currentMotion = Mode_Walk_0;
    executeMotion(currentMotion);
  } else if (command == "walk_backward") {
    currentMotion = Mode_Walk_180;
    executeMotion(currentMotion);
  } else if (command == "turn_left") {
    currentMotion = Mode_Turn_Left;
    executeMotion(currentMotion);
  } else if (command == "turn_right") {
    currentMotion = Mode_Turn_Right;
    executeMotion(currentMotion);
  } else {
    Serial.println("Unknown command. Available commands: standby, walk_forward, walk_backward, turn_left, turn_right.");
  }
}

void executeMotion(MotionMode mode) {
  switch (mode) {
    case Mode_Standby:
      setMotionLUT(lut_standby, lut_standby_length);
      break;
    case Mode_Walk_0:
      setMotionLUT(lut_walk_0, lut_walk_0_length);
      break;
    case Mode_Walk_180:
      setMotionLUT(lut_walk_180, lut_walk_180_length);
      break;
    case Mode_Turn_Left:
      setMotionLUT(lut_turn_left, lut_turn_left_length);
      break;
    case Mode_Turn_Right:
      setMotionLUT(lut_turn_right, lut_turn_right_length);
      break;
    default:
      Serial.println("Motion mode not implemented.");
      break;
  }
}

void setMotionLUT(int lut[][6][3], int length) {
  for (int step = 0; step < length; ++step) {
    for (int leg = 0; leg < 6; ++leg) {
      for (int joint = 0; joint < 3; ++joint) {
        int pwmValue = lut[step][leg][joint];
        pwm.setPWM(leg * 3 + joint, 0, pwmValue);
      }
    }
    delay(DELAY_MS);
  }
}
