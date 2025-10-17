#include <Wire.h>

// Flex sensor analog pins
const int pinkie_Data = A1;
const int finger_Data = A2;
const int thumb_Data = A3;

// Sensor readings
int pinkie = 0;
int finger = 0;
int thumb = 0;

// Calibration values (adjust as needed)
int thumb_high = 960;
int thumb_low = 870;
int finger_high = 915;
int finger_low = 827;
int pinkie_high = 950;
int pinkie_low = 850;

int response_time = 50;  // sampling delay (ms) — faster plot, smaller delay

void setup() {
  Serial.begin(9600);   // faster baud for smooth plot
  delay(500);
  Serial.println("thumb,finger,pinkie"); // column labels for Serial Plotter
}

void loop() {
  // Read sensor values
  pinkie = analogRead(pinkie_Data);
  finger = analogRead(finger_Data);
  thumb = analogRead(thumb_Data);

  // === Print in CSV format for Serial Plotter ===
  Serial.print(thumb);
  Serial.print(",");
  Serial.print(finger);
  Serial.print(",");
  Serial.println(pinkie);

  // (Optional) you can visualise thresholds too:
  // Serial.print(",");
  // Serial.print(thumb_high);
  // Serial.print(",");
  // Serial.print(thumb_low);
  // Serial.print(",");
  // Serial.print(finger_high);
  // Serial.print(",");
  // Serial.print(finger_low);
  // Serial.print(",");
  // Serial.print(pinkie_high);
  // Serial.print(",");
  // Serial.println(pinkie_low);

  delay(response_time);
}
