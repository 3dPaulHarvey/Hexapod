#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca(0x40);

// ===================== POTS (optional later) =====================
// Not used in this walk-cycle version, but left here for expansion.
#define POT_X A0
#define POT_Y A1
#define POT_Z A2

// ===================== PCA9685 SERVO CAL =====================
struct ServoCal {
  uint8_t ch;
  float centerPulse;       // ticks at true mechanical 90°
  float pulsesPerDegree;   // ticks/deg
  float pulseMin;
  float pulseMax;
  bool reversed;
};

// You observed all reversed: set all true.
// (Flip to false per joint if you re-verify directions.)
ServoCal hip   = {0, 310.5f, 1.433f, 150.0f, 650.0f, true};
ServoCal knee  = {1, 308.5f, 2.25f,  150.0f, 650.0f, true};
ServoCal ankle = {2, 311.5f, 1.41f,  150.0f, 650.0f, true};

// ===================== LEG GEOMETRY (mm) =====================
const float A = 76.0f;   // link A
const float B = 99.0f;   // link B
const float L = 52.0f;   // hip offset

// ===================== SIMPLE WALK CYCLE (mm, Hz) =====================
// Baby-step defaults. Increase stepAmpX/stepLiftZ slowly.
float yFixed     = 97.0f;     // lateral placement (Y out)
float zGround    = -110.0f;   // ground height (negative Z is down)
float xCenter    = 0.0f;      // neutral foot X under hip

float stepAmpX   = 75.0f;      // X half-amplitude (mm)  (try 5..15 first)
float stepLiftZ  = 105.0f;     // lift height (mm)       (try 5..25 first)
float gaitHz     = .5f;     // cycles/sec             (try 0.2..0.6 first)

float stanceSkew = 0.0f;      // optional: bias stance (mm)

// ===================== INTERNAL =====================
unsigned long t0;

// ---------- helpers ----------
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float acosSafe(float x) {
  return acosf(clampf(x, -1.0f, 1.0f));
}

static inline float angleToPulseFloat(const ServoCal& s, float angleDeg) {
  angleDeg = clampf(angleDeg, 0.0f, 180.0f);
  float delta = angleDeg - 90.0f;
  float pulse = s.centerPulse + (s.reversed ? -delta : +delta) * s.pulsesPerDegree;
  return clampf(pulse, s.pulseMin, s.pulseMax);
}

static inline uint16_t pulseToTicks(float pulse) {
  // PCA9685 register wants integer ticks; round-to-nearest
  return (uint16_t)(pulse + 0.5f);
}

// phase in [0,1)
static inline float phase01() {
  float t = (millis() - t0) / 1000.0f;
  float ph = fmodf(t * gaitHz, 1.0f);
  if (ph < 0) ph += 1.0f;
  return ph;
}

// Compute target foot position for a simple step cycle
void getWalkTarget(float &x, float &y, float &z) {
  float ph = phase01();
  y = yFixed;

  // [0..0.5) stance: foot on ground, moves backward relative to hip
  // [0.5..1) swing : foot lifts, moves forward
  if (ph < 0.5f) {
    float u = ph / 0.5f;                 // 0..1
    z = zGround;

    float xStart = xCenter + stepAmpX + stanceSkew;
    float xEnd   = xCenter - stepAmpX + stanceSkew;
    x = xStart + (xEnd - xStart) * u;
  } else {
    float u = (ph - 0.5f) / 0.5f;        // 0..1

    float xStart = xCenter - stepAmpX;
    float xEnd   = xCenter + stepAmpX;
    x = xStart + (xEnd - xStart) * u;

    // Lift arc: 0 at ends, max at mid
    float hump = sinf(u * PI);           // 0..1..0
    z = zGround + stepLiftZ * hump;
  }
}

// IK: yaw + planar 2-link with hip offset L
bool legIK(float x, float y, float z, float &theta1Deg, float &theta2Deg, float &theta3Deg) {
  float R = sqrtf(x * x + y * y);
  float theta1 = atan2f(y, x);           // radians

  float D = R - L;
  float down = -z;                       // z up; negative z is down
  float C = sqrtf(D * D + down * down);

  // Reachability
  const float Cmax = A + B;
  const float Cmin = fabsf(A - B);
  bool reachable = (C >= Cmin && C <= Cmax);

  // Clamp C to avoid NaNs
  float Cc = clampf(C, Cmin, Cmax);

  float g = atan2f(D, down);             // radians

  float b = acosSafe((A*A + Cc*Cc - B*B) / (2.0f * A * Cc));  // radians
  float h = acosSafe((A*A + B*B - Cc*Cc) / (2.0f * A * B));   // radians

  float theta2 = b + g;                  // radians
  float theta3 = (float)M_PI - h;        // radians (180° - h)

  theta1Deg = theta1 * 180.0f / PI;
  theta2Deg = theta2 * 180.0f / PI;
  theta3Deg = theta3 * 180.0f / PI;

  // Constrain to servo-friendly range (adjust if your joints use different limits)
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

  t0 = millis();

  // Serial Plotter header (may be ignored depending on IDE version; harmless)
  Serial.println("x\ty\tz\tt1\tt2\tt3\tp1\tp2\tp3\treach");

  // Print the commanded ranges once
  Serial.print("RANGES x=[");
  Serial.print(xCenter - stepAmpX, 1);
  Serial.print(",");
  Serial.print(xCenter + stepAmpX, 1);
  Serial.print("] y=");
  Serial.print(yFixed, 1);
  Serial.print(" zGround=");
  Serial.print(zGround, 1);
  Serial.print(" zSwingMax=");
  Serial.println(zGround + stepLiftZ, 1);
}

void loop() {
  // 1) Walk-cycle target
  float x, y, z;
  getWalkTarget(x, y, z);

  // 2) IK -> angles
  float t1, t2, t3;
  bool reach = legIK(x, y, z, t1, t2, t3);

  // 3) Angles -> pulses
  float p1 = angleToPulseFloat(hip,   t1);
  float p2 = angleToPulseFloat(knee,  t2);
  float p3 = angleToPulseFloat(ankle, t3);

  // 4) Drive PCA9685
  pca.setPWM(hip.ch,   0, pulseToTicks(p1));
  pca.setPWM(knee.ch,  0, pulseToTicks(p2));
  pca.setPWM(ankle.ch, 0, pulseToTicks(p3));

  // 5) Serial Plotter-friendly (numbers only, tab separated)
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

  delay(20); // ~50 Hz update
}
