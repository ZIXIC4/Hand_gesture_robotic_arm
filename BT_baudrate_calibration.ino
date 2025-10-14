#include <SoftwareSerial.h>
SoftwareSerial BTSerial(11, 10); // RX, TX

void setup() {
  Serial.begin(9600);
  Serial.println("=== HC-05 AT Mode Setup ===");
  Serial.println("Make sure the module is in AT mode (slow blinking LED)");
  
  // HC-05 AT模式固定波特率是38400
  BTSerial.begin(38400);
}

void loop() {
  // 电脑 → 蓝牙模块
  if (Serial.available()) {
    char c = Serial.read();
    BTSerial.write(c);
  }

  // 蓝牙模块 → 电脑
  if (BTSerial.available()) {
    char c = BTSerial.read();
    Serial.write(c);
  }
}
