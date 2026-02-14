#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca(0x40);

// ===== Pots for XYZ =====
#define POT_X A0
#define POT_Y A1
#define POT_Z A2

// ===== PCA channels: joint1/2/3 =====
struct ServoCal {
  uint8_t ch;
  float centerPulse;       // ticks at true mechanical 90°
  float pulsesPerDegree;   // ticks/deg
  float pulseMin;
  float pulseMax;
  bool reversed;
};

// You said all seem reversed -> set all true here
ServoCal hip   = {0, 310.5f, 1.433f, 150.0f, 650.0f, true};
ServoCal knee  = {1, 308.5f, 2.25f,  150.0f, 650.0f, true};
ServoCal ankle = {2, 311.5f, 1.41f,  150.0f, 650.0f, true};

// ===== IK geometry (mm) =====
const float A = 76.0f;   // link A
const float B = 99.0f;   // link B
const float L = 52.0f;   // hip offset

// ---------- helpers ----------
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float acosSafe(float x) {
  return acosf(clampf(x, -1.0f, 1.0f));
}

static inline float potToSigned100(uint8_t pin) {
  int raw = analogRead(pin);              // 0..1023
  // map to [-100, +100] as float with max resolution
  return (raw * (400.0f / 1023.0f)) - 200.0f;
}

static inline float angleToPulseFloat(const ServoCal& s, float angleDeg) {
  angleDeg = clampf(angleDeg, 0.0f, 180.0f);
  float delta = angleDeg - 90.0f;
  float pulse = s.centerPulse + (s.reversed ? -delta : +delta) * s.pulsesPerDegree;
  return clampf(pulse, s.pulseMin, s.pulseMax);
}

static inline uint16_t pulseToTicks(float pulse) {
  // PCA9685 expects integer tick counts; round-to-nearest
  return (uint16_t)(pulse + 0.5f);
}

// Returns true if reachable-ish; still provides clamped solution either way
bool legIK(float x, float y, float z, float &theta1Deg, float &theta2Deg, float &theta3Deg) {
  // Hip yaw
  float R = sqrtf(x * x + y * y);
  float theta1 = atan2f(y, x); // radians

  // Offset then planar solve in (D, -z)
  float D = R - L;
  float down = -z;             // with z up, negative z means down; down becomes positive
  float C = sqrtf(D * D + down * down);

  // Reachability checks for a 2-link arm
  const float Cmax = A + B;
  const float Cmin = fabsf(A - B);
  bool reachable = (C >= Cmin && C <= Cmax);

  // Clamp C slightly into [Cmin, Cmax] to avoid NaNs and keep motion stable
  float Cc = clampf(C, Cmin, Cmax);

  float g = atan2f(D, down);   // radians (matches your atan2(D, -z))

  // Law of cosines
  float b = acosSafe((A*A + Cc*Cc - B*B) / (2.0f * A * Cc));  // radians
  float h = acosSafe((A*A + B*B - Cc*Cc) / (2.0f * A * B));   // radians

  float theta2 = b + g;        // radians
  float theta3 = (float)M_PI - h; // radians; equivalent to 180 - h(deg)

  theta1Deg = theta1 * 180.0f / PI;
  theta2Deg = theta2 * 180.0f / PI;
  theta3Deg = theta3 * 180.0f / PI;

  // Optional: constrain to servo range expectations
  theta1Deg = clampf(theta1Deg, 0.0f, 180.0f);
  theta2Deg = clampf(theta2Deg, 0.0f, 180.0f);
  theta3Deg = clampf(theta3Deg, 0.0f, 180.0f);

  return reachable;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pca.begin();
  pca.setPWMFreq(50);

  // Serial Plotter header (safe if ignored)
  Serial.println("x\ty\tz\tt1\tt2\tt3\tp1\tp2\tp3\treach");
}

void loop() {
  // Pots -> XYZ in [-100, +100]
  float x = potToSigned100(POT_X);
  float y = potToSigned100(POT_Y);
  float z = potToSigned100(POT_Z);

  // IK -> angles
  float t1, t2, t3;
  bool reach = legIK(x, y, z, t1, t2, t3);

  // Angles -> pulses
  float p1 = angleToPulseFloat(hip,   t1);
  float p2 = angleToPulseFloat(knee,  t2);
  float p3 = angleToPulseFloat(ankle, t3);

  // Drive servos (int ticks at the last step)
  pca.setPWM(hip.ch,   0, pulseToTicks(p1));
  pca.setPWM(knee.ch,  0, pulseToTicks(p2));
  pca.setPWM(ankle.ch, 0, pulseToTicks(p3));

  // Plotter-friendly output: tab-separated numbers
  Serial.print(x, 1);   Serial.print('\t');
  Serial.print(y, 1);   Serial.print('\t');
  Serial.print(z, 1);   Serial.print('\t');
  Serial.print(t1, 1);  Serial.print('\t');
  Serial.print(t2, 1);  Serial.print('\t');
  Serial.print(t3, 1);  Serial.print('\t');
  Serial.print(p1, 1);  Serial.print('\t');
  Serial.print(p2, 1);  Serial.print('\t');
  Serial.print(p3, 1);  Serial.print('\t');
  Serial.println(reach ? 1 : 0);

  delay(20); // ~50Hz update for plotter
}
