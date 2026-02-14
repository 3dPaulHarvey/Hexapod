float theta1, theta2, theta3;
const float A = 76.0;
const float B = 99.0;
const float L = 52.0;


float x = 0.0;
float y = 0.0;
float z = 0.0;

void setup() {
  Serial.begin(115200);
  float R = sqrtf(x * x + y * y);
  theta1 = atan2(y, x);
  theta1 = theta1 * 180 / PI;

  float D = R - L;
  float g = atan2(D,-z);
  float C = sqrtf(D * D + (-z) *(-z));
  float b = acos((A * A + C * C - B * B) / (2 * A * C));
  theta2 = b + g;
  theta2 = theta2 * 180 / PI;

  float h = acos((A * A + B * B - C * C) / (2 * A * B));
  h = h * 180 / PI;
  theta3 = 180 - h;
}

void loop() {


}
