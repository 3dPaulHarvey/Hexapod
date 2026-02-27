
//hexapod x forward, y left, z up
//Servos calibrated to be 90 degrees centric
//looking down on the robot, the hip zeroed is straight back with plus degrees moving fowrad in x, forward on robot. the knee zero is straight down. with plus degrees being flex and minus degrees being extend. the foot is zeroed extending out vertically along with eh knee. for teh foot, t plus degres is flex and minus degrees is extension
//this will be for the right leg. the desired xyz is the tip of the foot. the L line is collinear, no funky twist or anything from the hip. standard setup. our servos are centered at 90 degrees mid, so we will be applying offsets to command them we are just testing one leg, so we will not include the body frame or other frames outside our system beginning at a single hip joint
//when all servos are at 90 degrees, hip points -y, knee points -y and foot points -z.
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
  float C = sqrtf((z * z) + (d * d));
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
