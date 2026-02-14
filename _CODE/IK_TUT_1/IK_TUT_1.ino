#include <Servo.h>
#include <math.h>

Servo coxaServo;   // Pin 6
Servo femurServo;  // Pin 10
Servo tibiaServo;  // Pin 11

// Link lengths
const float a = 200.0;  // Femur
const float b = 300.0;  // Tibia
const float l = 62.0;   // Coxa offset

void setup() {
  Serial.begin(9600);
  coxaServo.attach(6);
  femurServo.attach(10);
  tibiaServo.attach(11);
  Serial.println("Enter X Y Z (e.g., 100 100 100):");
}

void loop() {
  if (Serial.available()) {
    float x = Serial.parseFloat();
    float y = Serial.parseFloat();
    float z = Serial.parseFloat();

    delay(10);
    while (Serial.available()) Serial.read();

    float r = sqrt(x * x + y * y);  // switch r and d?
    float d = r - l;
    float c = sqrt(z * z + d * d);

    //Degrees
    float theta1 = atan2(y, x) * 180.0 / PI;
    float theta2 = atan2(d, -z) * 180.0 / PI
                   + acos((a * a + c * c - b * b) / (2 * a * c)) * 180.0 / PI;

    float theta3 = 180.0
                   - acos((a * a + b * b - c * c) / (2 * a * b)) * 180.0 / PI;

    // Print
    Serial.print("Theta1 (coxa): ");
    Serial.println(theta1);

    Serial.print("Theta2 (femur): ");
    Serial.println(theta2);

    Serial.print("Theta3 (tibia): ");
    Serial.println(theta3);

    Serial.println("----------------");

    // Move servos
    coxaServo.write(constrain(theta1, 0, 180));
    femurServo.write(constrain(theta2, 0, 180));
    tibiaServo.write(constrain(theta3, 0, 180));
  }
}
