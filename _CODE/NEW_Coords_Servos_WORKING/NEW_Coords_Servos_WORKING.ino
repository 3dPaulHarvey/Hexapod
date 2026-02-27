#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===================== TYPES =====================
struct ServoCal {
  uint8_t ch;
  float centerPulse;
  float pulsesPerDegree;
  float pulseMin;
  float pulseMax;
  bool reversed;
};

// ===================== PCA9685 / SERVOS =====================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Your calibrations (unchanged)
ServoCal hip  = {0, 349.0f, 2.2f,  150.0f, 650.0f, true};
ServoCal knee = {1, 329.0f, 2.24f, 150.0f, 650.0f, true};
ServoCal foot = {2, 297.0f, 2.14f, 150.0f, 650.0f, true};

// ===================== SETTINGS =====================
static const float START_DEG = 90.0f;     // start all at 90°
static const int MOVE_DELAY_MS = 500;     // small delay between moves
static const int LEG_DELAY_MS  = 1000;    // small delay between legs

// PCA9685 servo pulse helper
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float angleToPulse(const ServoCal& s, float angleDeg) {
  // Treat "90" as the center reference; offset is (angle - 90)
  float offsetDeg = angleDeg - 90.0f;
  if (s.reversed) offsetDeg = -offsetDeg;

  float pulse = s.centerPulse + offsetDeg * s.pulsesPerDegree;
  return clampf(pulse, s.pulseMin, s.pulseMax);
}

void writeServoDeg(const ServoCal& s, float angleDeg) {
  float pulse = angleToPulse(s, angleDeg);

  // PCA9685 expects 12-bit ticks (0..4095) for the current PWM frequency.
  // If your "pulse" values are already in PCA ticks (common approach), this is correct.
  // If your values are in microseconds, you'd need a us->ticks conversion instead.
  uint16_t ticks = (uint16_t)(pulse + 0.5f);

  pwm.setPWM(s.ch, 0, ticks);

  Serial.print("CH "); Serial.print(s.ch);
  Serial.print("  angle "); Serial.print(angleDeg, 1);
  Serial.print("  pulse/ticks "); Serial.println(ticks);
}

void testOneServo(const char* name, const ServoCal& s) {
  Serial.print("\n--- Testing "); Serial.print(name); Serial.println(" ---");

  // Start at 90
  writeServoDeg(s, START_DEG);
  delay(MOVE_DELAY_MS);
  

  // +/- 15
  writeServoDeg(s, START_DEG + 15.0f);
  delay(MOVE_DELAY_MS);
  writeServoDeg(s, START_DEG - 15.0f);
  delay(MOVE_DELAY_MS);

  // back to 90
  writeServoDeg(s, START_DEG);
  delay(MOVE_DELAY_MS);

  // +/- 45
  writeServoDeg(s, START_DEG + 45.0f);
  delay(MOVE_DELAY_MS);
  writeServoDeg(s, START_DEG - 45.0f);
  delay(MOVE_DELAY_MS);

  // end at 90
  writeServoDeg(s, START_DEG);
  delay(MOVE_DELAY_MS);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();

  pwm.begin();
  // Typical analog servo: 50–60 Hz. Use whatever you already use in your project.
  pwm.setPWMFreq(50);
  delay(10);

  Serial.println("Servo range test: start 90°, then +/-15°, then +/-45° (each servo separately).");
}

void loop() {
  // Do each "leg" (servo) separately with small delays between them
  testOneServo("HIP", hip);
  delay(LEG_DELAY_MS);

  testOneServo("KNEE", knee);
  delay(LEG_DELAY_MS);

  testOneServo("FOOT", foot);
  delay(LEG_DELAY_MS);

  // Pause before repeating the full sequence
  delay(2000);
}