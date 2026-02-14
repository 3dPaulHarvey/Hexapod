#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PCA9685 channels for ONE leg
const uint8_t HIP_CH   = 7;
const uint8_t KNEE_CH  = 8;
const uint8_t ANKLE_CH = 9;

/*
  IMPORTANT: Set these to your actual idle pulse values (counts 0-4095).
  These define the "standing default pose" you described.
*/
int HIP_IDLE   = 350;
int KNEE_IDLE  = 250;
int ANKLE_IDLE = 350;

/*
  Direction notes:
  - If a joint moves the wrong way, flip the sign of its STEP below.
  - Keep step sizes small at first.
*/

// How much each joint moves for the gait (tune these)
int HIP_STEP_FORWARD   = +70;  // hip swing forward
int HIP_STEP_BACK      = -70;  // hip swing back

int KNEE_LIFT_STEP     = +105;  // knee lift (bend) amount
int ANKLE_LIFT_STEP    = -75;  // ankle compensation while lifting

// Motion timing
const int STEP_DELAY_MS = 12;  // smoothness (smaller = faster updates)
const int HOLD_MS       = 120; // pause at key points

// Safety clamp (keep within sane servo range)
const int PULSE_MIN = 120;
const int PULSE_MAX = 650;

int clampPulse(int v) {
  if (v < PULSE_MIN) return PULSE_MIN;
  if (v > PULSE_MAX) return PULSE_MAX;
  return v;
}

void setJoint(uint8_t ch, int pulse) {
  pwm.setPWM(ch, 0, clampPulse(pulse));
}

void setPose(int hip, int knee, int ankle) {
  setJoint(HIP_CH, hip);
  setJoint(KNEE_CH, knee);
  setJoint(ANKLE_CH, ankle);
}

// Linear move all 3 joints together over N steps (blocking but simple)
void moveTo(int hipTarget, int kneeTarget, int ankleTarget, int steps) {
  // Read current by tracking locally (we’ll keep static current pose)
  static int hipCur = 0, kneeCur = 0, ankleCur = 0;
  static bool init = false;

  if (!init) {
    hipCur = HIP_IDLE; kneeCur = KNEE_IDLE; ankleCur = ANKLE_IDLE;
    init = true;
    setPose(hipCur, kneeCur, ankleCur);
    delay(200);
  }

  float hipInc   = (hipTarget   - hipCur)   / (float)steps;
  float kneeInc  = (kneeTarget  - kneeCur)  / (float)steps;
  float ankleInc = (ankleTarget - ankleCur) / (float)steps;

  for (int i = 0; i < steps; i++) {
    hipCur   = (int)(hipCur   + hipInc);
    kneeCur  = (int)(kneeCur  + kneeInc);
    ankleCur = (int)(ankleCur + ankleInc);
    setPose(hipCur, kneeCur, ankleCur);
    delay(STEP_DELAY_MS);
  }

  // Snap exactly to target
  hipCur = hipTarget; kneeCur = kneeTarget; ankleCur = ankleTarget;
  setPose(hipCur, kneeCur, ankleCur);
}

void standIdle() {
  moveTo(HIP_IDLE, KNEE_IDLE, ANKLE_IDLE, 25);
  delay(HOLD_MS);
}

/*
  Very simple single-leg “walk-like” cycle:

  1) Lift foot: bend knee + adjust ankle
  2) Swing forward: hip forward while lifted
  3) Lower foot: return knee/ankle to idle
  4) Push back: hip back while on ground (simulates stance phase)
  5) Return to center
*/
void gaitCycle() {
  // Targets derived from idle + steps (tune per your mechanics)
  int hipCenter = HIP_IDLE;
  int hipFwd    = HIP_IDLE  + HIP_STEP_FORWARD;
  int hipBack   = HIP_IDLE  + HIP_STEP_BACK;

  int kneeLift  = KNEE_IDLE + KNEE_LIFT_STEP;
  int ankleLift = ANKLE_IDLE + ANKLE_LIFT_STEP;

  // 1) Lift
  moveTo(hipCenter, kneeLift, ankleLift, 20);
  delay(HOLD_MS);

  // 2) Swing forward while lifted
  moveTo(hipFwd, kneeLift, ankleLift, 25);
  delay(HOLD_MS);

  // 3) Lower to idle posture (foot down)
  moveTo(hipFwd, KNEE_IDLE, ANKLE_IDLE, 20);
  delay(HOLD_MS);

  // 4) Push back on ground
  moveTo(hipBack, KNEE_IDLE, ANKLE_IDLE, 35);
  delay(HOLD_MS);

  // 5) Return to center
  moveTo(hipCenter, KNEE_IDLE, ANKLE_IDLE, 25);
  delay(HOLD_MS);
}

void setup() {
  Serial.begin(115200);
  delay(50);

  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);

  // Start in default standing pose
  standIdle();

  Serial.println("Single-leg gait test running. Tune *_IDLE and STEP values.");
}

void loop() {
  gaitCycle();
}
