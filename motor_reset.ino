#include <Servo.h>

Servo servo1;
Servo servo2;
void setup() {
  servo1.attach(9);  // Connect servo signal wire to digital pin 9
  servo2.attach(10);
}

void loop() {
  // Blue base servo (view from the bottom in the clock wise direction)
  // White base servo (view from the bottom in the clock wise direction)
 
  servo1.writeMicroseconds(500);  // Move to 0 degrees 
  servo2.writeMicroseconds(2500); // Move to 270 degrees 
  delay(1000);

  servo1.writeMicroseconds(1167);  // Move to 90 degrees 
  servo2.writeMicroseconds(1833);  // Move to 180 degrees 
  delay(1000);
}

