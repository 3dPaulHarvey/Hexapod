#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===================== PCA9685 SERVO CAL =====================
struct ServoCal {
  uint8_t ch;
  float centerPulse;       // ticks at true mechanical 90°
  float pulsesPerDegree;   // ticks/deg
  float pulseMin;          // clamp min ticks
  float pulseMax;          // clamp max ticks
  bool reversed;
};

// Your cal (ticks are PCA9685 "counts" 0..4095 per 20ms frame at 50Hz)
ServoCal hip  = {0, 349.0f, 2.2f,  150.0f, 650.0f, true};
ServoCal knee = {1, 329.0f, 2.24f, 150.0f, 650.0f, true};

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Convert a requested angle in degrees where 90 = center,
// into PCA9685 ticks using your calibration.
uint16_t angleToTicks(const ServoCal &s, float angleDeg) {
  // We treat 90° as the center reference in your calibration.
  float delta = angleDeg - 90.0f;        // degrees away from center
  if (s.reversed) delta = -delta;

  float ticks = s.centerPulse + (delta * s.pulsesPerDegree);
  ticks = clampf(ticks, s.pulseMin, s.pulseMax);

  return (uint16_t)(ticks + 0.5f); // round to nearest int
}

void setServoAngle(const ServoCal &s, float angleDeg) {
  uint16_t ticks = angleToTicks(s, angleDeg);
  // PCA9685: setPWM(channel, on_tick, off_tick)
  // We use ON=0 and OFF=ticks for a simple pulse each cycle.
  pca.setPWM(s.ch, 0, ticks);

  Serial.print("CH ");
  Serial.print(s.ch);
  Serial.print(" angle ");
  Serial.print(angleDeg, 1);
  Serial.print(" -> ticks ");
  Serial.println(ticks);
}

// Move one servo through: center, +15, center, -15, center, +45, center, -45, center
void testOneServo(const char *name, const ServoCal &s, uint16_t holdMs = 1200) {
  Serial.println();
  Serial.print("=== Testing ");
  Serial.print(name);
  Serial.print(" (CH ");
  Serial.print(s.ch);
  Serial.println(") ===");

  const float center = 90.0f;

  setServoAngle(s, center);        delay(holdMs);

  setServoAngle(s, center + 15.0f); delay(holdMs);
  setServoAngle(s, center);         delay(holdMs);

  setServoAngle(s, center - 15.0f); delay(holdMs);
  setServoAngle(s, center);         delay(holdMs);

  setServoAngle(s, center + 45.0f); delay(holdMs);
  setServoAngle(s, center);         delay(holdMs);

  setServoAngle(s, center - 45.0f); delay(holdMs);
  setServoAngle(s, center);         delay(holdMs);

  Serial.print("=== Done ");
  Serial.print(name);
  Serial.println(" ===");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("PCA9685 Servo Cal Test starting...");

  Wire.begin();
  pca.begin();

  // Standard servo refresh rate
  pca.setPWMFreq(50);
  delay(10);

  // Start both at center (optional, helps avoid surprises)
  setServoAngle(hip, 90.0f);
  setServoAngle(knee, 90.0f);
  delay(1500);

  Serial.println("Setup complete.");
}

void loop() {
  // Do each movement separately (one servo at a time)
  testOneServo("HIP", hip);
  delay(1500);

  testOneServo("KNEE", knee);
  delay(3000);
}