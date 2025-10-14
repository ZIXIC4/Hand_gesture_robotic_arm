#include <Servo.h>

Servo servo;

void setup() {
  servo.attach(9);  // Connect servo signal wire to digital pin 9
}

void loop() {
  // view from the bottom, rotate clock-wsie direction
  // Move to 90 degrees (1ms pulse)
  // servo.writeMicroseconds(500);
  // delay(1000);

  // Move to 0 degrees (1.5ms pulse, middle)
  servo.writeMicroseconds(1500);
  delay(1000);

  // // Move to -90 degrees (2ms pulse)
  // servo.writeMicroseconds(2500);
  // delay(1000);
}
