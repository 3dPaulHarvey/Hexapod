
//hexapod x forward, y left, z up
const float A = 76.0f;  // Knee Link
const float B = 99.0f;  // Foot Link
const float L = 52.0f;  // hip -> knee offset

float x, y, z = 0;
float hipAngle, kneeAngle, footAngle;
void setup() {
  Serial.begin(115200);
}

void loop() {
  x = 0;
  y = 170;
  z = -5;

  Serial.println(x);
  Serial.println(y);
  Serial.println(z);
  Serial.println();

  float r = sqrtf((x * x) + (y * y));
  hipAngle = atan2(y, x);
  hipAngle = degrees(hipAngle);
  Serial.println(hipAngle);

  float d = r - L;
  if (d < 0.0f) d = 0.0f; // clamp for too close
  float C = sqrtf((z * z) + (d * d));
  if (C > (A + B)) { Serial.println("unreachable"); } //too far
  if (C < (fabsf(A - B))) { Serial.println("unreachable"); } //too close

  float a = atan2(d, -z);

  float b = ((A * A) + (C * C) - (B * B)) / (2 * A * C);
  b = constrain(b, -1.0f, 1.0f);;
  b = acosf(b);
  kneeAngle = a + b;
  kneeAngle = degrees(kneeAngle);
  Serial.println(kneeAngle);

  float g = ((A * A) + (B * B) - (C * C)) / (2 * A * B);
  g = constrain(g, -1.0f, 1.0f);;
  g = acosf(g);
  footAngle = PI - g;
  footAngle = degrees(footAngle);

  Serial.println(footAngle);

  Serial.println();

  Serial.println();
  delay(2000);
}
