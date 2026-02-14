
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PCA9685 channels for ONE leg
const uint8_t HIP_CH   = 7;
const uint8_t KNEE_CH  = 8;
const uint8_t ELBOW_CH = 9;

//idle pulse length
int HIP_IDLE   = 350;
int KNEE_IDLE  = 250;
int ANKLE_IDLE = 350;

// Link lengths (IK)
const float a = 200.0;  // Femur
const float b = 300.0;  // Tibia
const float l = 62.0;   // Coxa offset

float clampUnit(float v) {
  if (v < -1.0f) return -1.0f;
  if (v >  1.0f) return  1.0f;
  return v;
}

bool computeIK(float x, float y, float z, float &hipDeg, float &kneeDeg, float &elbowDeg) {
  float r = sqrtf(x * x + y * y);
  float d = r - l;
  float c = sqrtf(z * z + d * d);

  if (c < 1e-6f) return false;

  // Degrees
  float theta1 = atan2f(y, x) * 180.0f / PI;

  float cosA = clampUnit((a * a + c * c - b * b) / (2.0f * a * c));
  float theta2 = atan2f(d, -z) * 180.0f / PI + acosf(cosA) * 180.0f / PI;

  float cosB = clampUnit((a * a + b * b - c * c) / (2.0f * a * b));
  float theta3 = 180.0f - acosf(cosB) * 180.0f / PI;

  hipDeg   = theta1;
  kneeDeg  = theta2;
  elbowDeg = theta3;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(50);

  // PCA9685 init (kept, but we do not command any servos)
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);

  Serial.println("Bare-bones IK (Hip/Knee/Elbow) ready.");
  Serial.println("Enter X Y Z (e.g., 100 100 100):");
}

void loop() {
  if (!Serial.available()) return;

  float x = Serial.parseFloat();
  float y = Serial.parseFloat();
  float z = Serial.parseFloat();

  delay(10);
  while (Serial.available()) Serial.read();

  float hipDeg, kneeDeg, elbowDeg;
  if (!computeIK(x, y, z, hipDeg, kneeDeg, elbowDeg)) {
    Serial.println("IK failed (unreachable/invalid).");
    return;
  }

  Serial.print("XYZ: "); Serial.print(x); Serial.print(" ");
  Serial.print(y); Serial.print(" "); Serial.println(z);

  Serial.print("Hip: ");   Serial.println(hipDeg);
  Serial.print("Knee: ");  Serial.println(kneeDeg);
  Serial.print("Elbow: "); Serial.println(elbowDeg);
  Serial.println("----------------");
}
