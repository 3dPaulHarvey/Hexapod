// Drop-in “draw a square” mode for your single right leg.
// Uses the SAME solveIK_andDrive() + servo setup you already have.
// Foot traces a square in the X-Y plane at a fixed Z (like drawing on paper).

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

struct ServoCal {
  uint8_t ch;
  float centerPulse;
  float pulsesPerDegree;
  float pulseMin;
  float pulseMax;
  bool reversed;
};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Cal (unchanged)
ServoCal hip = { 0, 349.0f, 2.2f, 150.0f, 650.0f, true };
ServoCal knee = { 1, 329.0f, 2.24f, 150.0f, 650.0f, true };
ServoCal foot = { 2, 297.0f, 2.14f, 150.0f, 650.0f, true };

// Geometry (mm)
const float A = 76.0f;
const float B = 99.0f;
const float L = 52.0f;

static const int PWM_FREQ_HZ = 50;
static const uint16_t LOOP_MS = 20;

// ===================== SQUARE PARAMS =====================
// Center of square (mm) in hip frame (+x forward, +y left, +z up)
float cx = 0.0f;
float cy = -160.0f;

// "Pen down" height (mm). Tune so it just touches your table/paper.
float zDraw = -40.0f;

// Side length (mm) of the square
float side = 60.0f;

// Time to complete one full square (seconds)
float squarePeriodSec = 3.0f;

// Corner rounding (0..0.49). 0 = sharp corners. ~0.10 looks nicer.
float cornerRound = 0.10f;

// ===================== HELPERS =====================
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static inline float rad2deg(float r) {
  return r * (180.0f / PI);
}
static inline float lerpf(float a, float b, float t) {
  return a + (b - a) * t;
}
static inline float smoothstep(float t) {
  t = clampf(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

float angleToPulse(const ServoCal& s, float angleDeg) {
  float offsetDeg = angleDeg - 90.0f;
  if (s.reversed) offsetDeg = -offsetDeg;
  float pulse = s.centerPulse + offsetDeg * s.pulsesPerDegree;
  return clampf(pulse, s.pulseMin, s.pulseMax);
}

void writeServoDeg(const ServoCal& s, float angleDeg) {
  float pulse = angleToPulse(s, angleDeg);
  uint16_t ticks = (uint16_t)(pulse + 0.5f);
  pwm.setPWM(s.ch, 0, ticks);
}

// ===================== IK =====================
void solveIK_andDrive(float x, float y, float z) {
  float thetaDeg = rad2deg(atan2f(y, x));
  float hipServoDeg = 180.0f + thetaDeg;

  float r = sqrtf((x * x) + (y * y));
  float d = r - L;
  float C = sqrtf((z * z) + (d * d));

  float a = atan2f(d, -z);
  float b = ((A * A) + (C * C) - (B * B)) / (2.0f * A * C);
  b = clampf(b, -1.0f, 1.0f);
  b = acosf(b);
  float kneeAngleDeg = rad2deg(a + b);

  float g = ((A * A) + (B * B) - (C * C)) / (2.0f * A * B);
  g = clampf(g, -1.0f, 1.0f);
  g = acosf(g);
  float footAngleDeg = rad2deg(PI - g);

  writeServoDeg(hip, hipServoDeg);
  writeServoDeg(knee, kneeAngleDeg);
  writeServoDeg(foot, footAngleDeg);
}

// ===================== SQUARE TRAJECTORY =====================
// Parameter u in [0..1) around the square.
// We generate a square centered at (cx,cy) with side length `side`.
// We also optionally round corners with `cornerRound`.
void squareXY(float u, float& x, float& y) {
  u = u - floorf(u);  // wrap
  float half = 0.5f * side;

  // Split into 4 edges: each spans 0.25 of u
  float seg = u * 4.0f;      // [0..4)
  int i = (int)floorf(seg);  // 0,1,2,3
  float t = seg - (float)i;  // [0..1)

  // Corner rounding: ease near ends of each segment
  float cr = clampf(cornerRound, 0.0f, 0.49f);
  if (cr > 0.0f) {
    // Map t with eased ends: [0..cr] eased, [cr..1-cr] linear, [1-cr..1] eased
    if (t < cr) {
      float tt = t / cr;
      t = cr * smoothstep(tt);
    } else if (t > 1.0f - cr) {
      float tt = (t - (1.0f - cr)) / cr;
      t = (1.0f - cr) + cr * smoothstep(tt);
    }
  }

  // Edges in order (clockwise):
  // 0: top    (x: -half -> +half, y: +half)
  // 1: right  (x: +half, y: +half -> -half)
  // 2: bottom (x: +half -> -half, y: -half)
  // 3: left   (x: -half, y: -half -> +half)
  float ex = 0, ey = 0;
  switch (i & 3) {
    case 0:
      ex = lerpf(-half, +half, t);
      ey = +half;
      break;
    case 1:
      ex = +half;
      ey = lerpf(+half, -half, t);
      break;
    case 2:
      ex = lerpf(+half, -half, t);
      ey = -half;
      break;
    default:
      ex = -half;
      ey = lerpf(-half, +half, t);
      break;
  }

  x = cx + ex;
  y = cy + ey;
}

float phaseFromMillis(uint32_t ms, float periodSec) {
  float t = (float)ms * 0.001f;
  float p = t / periodSec;
  return p - floorf(p);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ_HZ);
  delay(10);

  Serial.println("Square draw mode: traces a square in X-Y at fixed Z.");
}

void loop() {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if ((uint32_t)(now - lastMs) < LOOP_MS) return;
  lastMs = now;

  float u = phaseFromMillis(now, squarePeriodSec);

  float x, y;
  squareXY(u, x, y);
  float z = zDraw;

  // Debug occasionally
  static uint32_t dbg = 0;
  if (now - dbg > 250) {
    dbg = now;
    Serial.print("u=");
    Serial.print(u, 3);
    Serial.print("  xyz=");
    Serial.print(x, 1);
    Serial.print(", ");
    Serial.print(y, 1);
    Serial.print(", ");
    Serial.println(z, 1);
  }

  solveIK_andDrive(x, y, z);
}