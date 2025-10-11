/******************************************************************************
  Author: Smartbuilds.io
  YouTube: https://www.youtube.com/channel/UCGxwyXJWEarxh2XWqvygiIg
  Fork your own version: https://github.com/EbenKouao/arduino-robotic-arm
  Date: 28/12/2020
  Description: Robotic Glove to control the Robotic Arm using Bluetooth

******************************************************************************/

#include<Wire.h>


// MPU1 -> Palm movement； MPU2 -> Upper arm movement
//const int MPU_addr = 0x68;
const int MPU2 = 0x69, MPU1 = 0x68;

//Ac -> Line acceleartion; Gu -> Angular velocity 

//First MPU6050
int16_t AcX1, AcY1, AcZ1, Tmp1, GyX1, GyY1, GyZ1; 
int minVal = 265;
int maxVal = 402;
double x;
double y;
double z;

//Second MPU6050
int16_t AcX2, AcY2, AcZ2, Tmp2, GyX2, GyY2, GyZ2;
int minVal2 = 265;
int maxVal2 = 402;
double x2;
double y2;
double z2;


//How often to send values to the Robotic Arm
int response_time = 100;

void setup() {
  pinMode(3, OUTPUT);
  Wire.begin();
  Wire.beginTransmission(MPU1);
  Wire.write(0x6B);// PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true); 
  // Wire.begin();
  // Wire.beginTransmission(MPU2);
  // Wire.write(0x6B);// PWR_MGMT_1 register
  // Wire.write(0); // set to zero (wakes up the MPU-6050)
  // Wire.endTransmission(true);
  Serial.begin(4800);
  delay(1000);

}
void loop() {

  /*
    Note: Serial.print() would send all values the robotic arm using via bluetooth.
  */
  digitalWrite(3, HIGH); //Use basic LED as visual indicator if value being sent

  // debug_flex(); //Debug Mode on/off
  // Serial.println("test");
  //get values for first mpu having address of 0x68
  GetMpuValue1(MPU1);
  Serial.print(x);  Serial.print(" ");Serial.print(y);Serial.print(" ");  Serial.print(z);
  Serial.print(" ");
  GetMpuValue2(MPU2);
  Serial.print(x2);  Serial.print(" ");Serial.print(y2);Serial.print(" ");  Serial.println(z2);
  delay(100);
 
}

void GetMpuValue1(const int MPU) {

  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers

  AcX1 = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY1 = Wire.read() << 8 |  Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ1 = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp1 = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  // int xAng = map(AcX1, minVal, maxVal, -90, 90); 
  // int yAng = map(AcY1, minVal, maxVal, -90, 90);
  // int zAng = map(AcZ1, minVal, maxVal, -90, 90);

  GyX1 = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY1 = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L) 
  GyZ1 = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI) + 4; //offset by 4 degrees to get back to zero
  // y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  // z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  Serial.print(AcX1);
  Serial.print(" ");
  Serial.print(AcY1);
  Serial.print(" ");
  Serial.print(AcZ1);
  Serial.print(" ");
  Serial.print(GyX1);
  Serial.print(" ");
  Serial.print(GyY1);
  Serial.print(" ");
  Serial.println(GyZ1);

}

void GetMpuValue2(const int MPU) {

  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // request a total of 14 registers
  AcX2 = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY2 = Wire.read() << 8 |  Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ2 = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  Tmp2 = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  // int xAng2 = map(AcX2, minVal2, maxVal2, -90, 90);
  // int yAng2 = map(AcY2, minVal2, maxVal2, -90, 90);
  // int zAng2 = map(AcZ2, minVal2, maxVal2, -90, 90);

  GyX2 = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY2 = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ2 = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // x2 = RAD_TO_DEG * (atan2(-yAng2, -zAng2) + PI) + 4; //offset by 4 degrees to get back to zero
  // y2 = RAD_TO_DEG * (atan2(-xAng2, -zAng2) + PI);
  // z2 = RAD_TO_DEG * (atan2(-yAng2, -xAng2) + PI);


  Serial.print(AcX2);
  Serial.print(" ");
  Serial.print(AcY2);
  Serial.print(" ");
  Serial.print(AcZ2);
  Serial.print(" ");
  Serial.print(GyX2);
  Serial.print(" ");
  Serial.print(GyY2);
  Serial.print(" ");
  Serial.println(GyZ2);
}