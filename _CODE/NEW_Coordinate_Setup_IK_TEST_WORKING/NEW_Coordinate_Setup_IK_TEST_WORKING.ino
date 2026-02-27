//hexapod x forward, y left, z up
//Servos calibrated to be 90 degrees centric
//looking down on the robot, the hip zeroed is straight back with plus degrees moving fowrad in x, forward on robot. the knee zero is straight down. with plus degrees being flex and minus degrees being extend. the foot is zeroed extending out vertically along with eh knee. for teh foot, t plus degres is flex and minus degrees is extension
//this will be for the right leg. the desired xyz is the tip of the foot. the L line is collinear, no funky twist or anything from the hip. standard setup. our servos are centered at 90 degrees mid, so we will be applying offsets to command them we are just testing one leg, so we will not include the body frame or other frames outside our system beginning at a single hip joint
//when all servos are at 90 degrees, hip points -y, knee points -y and foot points -z.

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
ServoCal hip = { 0, 349.0f, 2.2f, 150.0f, 650.0f, true };
ServoCal knee = { 1, 329.0f, 2.24f, 150.0f, 650.0f, true };
ServoCal foot = { 2, 297.0f, 2.14f, 150.0f, 650.0f, true };

// ===================== GEOMETRY (mm) =====================
// A: knee link, B: foot link, L: hip->knee offset (collinear)
const float A = 76.0f;
const float B = 99.0f;
const float L = 52.0f;

// ===================== SETTINGS =====================
static const int PWM_FREQ_HZ = 50;
static const int LOOP_MS = 50;

// Desired foot tip target in THIS single-leg hip frame
// +x forward, +y left, +z up
// Right leg workspace is typically y < 0
float x = 30.0f;
float y = -160.0f;
float z = -20.0f;

// ===================== HELPERS =====================
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float rad2deg(float r) {
  return r * (180.0f / PI);
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
  uint16_t ticks = (uint16_t)(pulse + 0.5f);
  pwm.setPWM(s.ch, 0, ticks);

  Serial.print("CH ");
  Serial.print(s.ch);
  Serial.print("  ang ");
  Serial.print(angleDeg, 2);
  Serial.print("  ticks ");
  Serial.println(ticks);
}

// ===================== IK =====================
// Returns servo degrees (NOT math angles), using your 90° mechanical pose:
// hip@90 -> -y, knee@90 -> -y, foot@90 -> -z
void solveIK_andDrive(float x, float y, float z) {
  // --- Hip yaw ---
  // theta = atan2(y, x): 0 at +x, +90 at +y, -90 at -y
  float thetaDeg = rad2deg(atan2f(y, x));

  // Right-leg friendly mapping so that:
  // point along -y => theta=-90 => hipServo=90
  // hipServoDeg = 180 + thetaDeg
  float hipServoDeg = 180.0f + thetaDeg;

  // --- Planar triangle for knee/foot (matches your diagram) ---
  float r = sqrtf((x * x) + (y * y));
  float d = r - L;
  float C = sqrtf((z * z) + (d * d));

  // Knee: phi2 = a + b, where a = atan2(d, -z)
  float a = atan2f(d, -z);
  float b = ((A * A) + (C * C) - (B * B)) / (2.0f * A * C);
  b = clampf(b, -1.0f, 1.0f);
  b = acosf(b);
  float kneeAngleDeg = rad2deg(a + b);

  // Foot: phi3 = pi - g
  float g = ((A * A) + (B * B) - (C * C)) / (2.0f * A * B);
  g = clampf(g, -1.0f, 1.0f);
  g = acosf(g);
  float footAngleDeg = rad2deg(PI - g);

  // Map IK angles directly to servo degrees for your stated 90° pose:
  // - kneeAngleDeg = 90 when knee link is horizontal outward -> matches knee servo 90 points -y
  // - footAngleDeg = 90 when foot is down from a horizontal knee link -> matches foot servo 90 points -z
  float kneeServoDeg = kneeAngleDeg;
  float footServoDeg = footAngleDeg;

  // --- Print what should happen ---
  Serial.println();
  Serial.print("Target (x,y,z): ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z);

  Serial.print("Expected: hip yaws to aim at (x,y). ");
  if (y < 0) Serial.println("Since y<0 (right side), leg should generally point toward -y.");
  else Serial.println("y>=0 moves leg toward left side (+y).");

  Serial.print("thetaDeg = ");
  Serial.print(thetaDeg, 2);
  Serial.print("  hipServoDeg = ");
  Serial.println(hipServoDeg, 2);

  Serial.print("kneeAngleDeg = ");
  Serial.print(kneeAngleDeg, 2);
  Serial.print("  footAngleDeg = ");
  Serial.println(footAngleDeg, 2);

  Serial.println("Expected: increasing kneeAngle bends/lifts; footAngle ~90 means foot points down with knee roughly horizontal.");
  Serial.println();

  // --- Drive servos ---
  writeServoDeg(hip, hipServoDeg);
  writeServoDeg(knee, kneeServoDeg);
  writeServoDeg(foot, footServoDeg);
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ_HZ);
  delay(10);

  Serial.println("Right-leg IK test using PCA9685");
  Serial.println("90° pose: hip->-y, knee->-y, foot->-z");
  Serial.println("Mapping: hipServo=180+atan2(y,x)deg, kneeServo=kneeAngle, footServo=footAngle");
  Serial.println();
}

void loop() {
  // Stationary IK test point:
  // Try adjusting x forward/back and y more/less negative to see hip yaw changes.

  //   float x = 30.0f;
  // float y = -160.0f;
  // float z = -20.0f;
  x = 0.0f;
  y = -160.0f;
  z = -20.0f;
  solveIK_andDrive(x, y, z);
  delay(LOOP_MS);
  delay(2000);
  Serial.println("DOOOOOOOOOOOOOOOOOOOOWN");
  x = 00.0f;
  y = -160.0f;
  z = -50.0f;
  solveIK_andDrive(x, y, z);
  delay(LOOP_MS);
    delay(2000);
}