//Structure
float A = 200.0;
float B = 300.0;
float L = 62.0;

//Desire
float x, y, z;
float theta1, theta2, theta3;

void setup() {
  x = 00.0;
  y = 200.0;
  z = 100.0;

  Serial.begin(115200);
  theta1 = atan2(y, x);

  float R = sqrt(x * x + y * y);
  float D = R - L;
  float C = sqrt(D * D + -z * -z);  //Leg beneath origin -z
  theta2 = atan2(D, -z) + acos((A * A + C * C - B * B) / (2 * A * C));

  theta3 = (acos((A * A + B * B - C * C) / (2 * A * B)));

  theta1 = theta1 * 180 / PI;
  theta2 = theta2 * 180 / PI;
  theta3 = theta3 * 180 / PI;
  theta3 = 180 - theta3;
}


void loop() {
  //hip, knee, ankle
  Serial.print(x);
  Serial.print(y);
  Serial.print(z);
  Serial.println();
  Serial.println(theta1);
  Serial.println(theta2);
  Serial.println(theta3);
  Serial.println();
  delay(10000);
}
