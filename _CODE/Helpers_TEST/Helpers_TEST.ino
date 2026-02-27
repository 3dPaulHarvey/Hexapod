#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

struct ServoCal {
  uint8_t ch;
  float centerPulse;
  float pulsesPerDegree;
  float minPulse;
  float maxPulse;
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

//HELPERS
static float clampf(float x, float low, float hi) {
  if (x >= hi) {
    return hi;
  }
  if (x <= low) {
    return low;
  }
  return x;
}

static float rad2deg(float x) {
  return x * 180.0f / PI;
}

static float lerpf(float start, float end, float t) {
  return start + t * (end - start);
}

void writeServoDegree(const ServoCal& servo, float deg) {
  float delta = deg - 90;
  if (servo.reversed) { delta = -delta; }
  float offset = delta * servo.pulsesPerDegree;
  float pulse  = servo.centerPulse + offset;
  pulse = clampf(pulse, servo.minPulse, servo.maxPulse);
  uint16_t ticks = (uint16_t)(pulse + 0.5f);

  pwm.setPWM(servo.ch, 0, ticks);
}
/////////////IK
void solveIK_andDrive(float x, float y, float z){
  
}



//////////////////SETUP
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ_HZ);
  delay(10);

}

void loop() {
  
}
