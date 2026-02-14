#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca(0x40);

// Pots
#define POT_HIP   A0
#define POT_KNEE  A1
#define POT_ANKLE A2

// Safety limits (ticks)
const float PULSE_MIN = 150.0f;
const float PULSE_MAX = 650.0f;

struct ServoCal {
  uint8_t ch;
  uint8_t potPin;
  float centerPulse;       // ticks at true mechanical 90°
  float pulsesPerDegree;   // ticks/deg
  bool reversed;
};

// Your joints
ServoCal hip   = {0, POT_HIP,   310.5f, 1.433f, true};
ServoCal knee  = {1, POT_KNEE,  308.5f, 2.25f,  true };
ServoCal ankle = {2, POT_ANKLE, 311.5f, 1.41f,  true };

static inline float potToAngleDeg(uint8_t potPin) {
  int raw = analogRead(potPin);             // 0..1023
  return (raw * 180.0f) / 1023.0f;          // float degrees
}

static inline float angleToPulseFloat(const ServoCal& s, float angleDeg) {
  angleDeg = constrain(angleDeg, 0.0f, 180.0f);
  float delta = angleDeg - 90.0f;

  float pulse = s.centerPulse + (s.reversed ? -delta : +delta) * s.pulsesPerDegree;
  return constrain(pulse, PULSE_MIN, PULSE_MAX);
}

static inline uint16_t pulseToTicks(float pulse) {
  // PCA register needs integer ticks; round-to-nearest
  return (uint16_t)(pulse + 0.5f);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pca.begin();
  pca.setPWMFreq(50);

  // Serial Plotter: optional header (some IDE versions ignore it; fine either way)
  Serial.println("hip_deg\tknee_deg\tankle_deg\thip_p\tknee_p\tankle_p");
}

void loop() {
  // Read angles (float)
  float hipDeg   = potToAngleDeg(hip.potPin);
  float kneeDeg  = potToAngleDeg(knee.potPin);
  float ankleDeg = potToAngleDeg(ankle.potPin);

  // Compute pulses (float)
  float hipP   = angleToPulseFloat(hip, hipDeg);
  float kneeP  = angleToPulseFloat(knee, kneeDeg);
  float ankleP = angleToPulseFloat(ankle, ankleDeg);

  // Drive PCA (int ticks at the last step)
  pca.setPWM(hip.ch,   0, pulseToTicks(hipP));
  pca.setPWM(knee.ch,  0, pulseToTicks(kneeP));
  pca.setPWM(ankle.ch, 0, pulseToTicks(ankleP));

  // Serial Plotter output: numbers only, tab-separated
  Serial.print(hipDeg, 2);   Serial.print('\t');
  Serial.print(kneeDeg, 2);  Serial.print('\t');
  Serial.print(ankleDeg, 2); Serial.print('\t');
  Serial.print(hipP, 1);     Serial.print('\t');
  Serial.print(kneeP, 1);    Serial.print('\t');
  Serial.println(ankleP, 1);

  delay(20); // ~50 Hz plot update; raise if plot is too dense
}
