#include<Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial BT(11, 10); // nano's RX, TX


//Create thumb Sensors
int pinkie = 0; //Pinkie thumb
int finger = 0; //finger thumb
int thumb = 0; //Index thumb

int pinkie_Data = A1;
int finger_Data = A2;
int thumb_Data = A3;

// MPU1 -> Palm movement； MPU2 -> Upper arm movement
//const int MPU_addr = 0x68;
const int MPU2 = 0x69, MPU1 = 0x68;
const int SMOOTH_WINDOW = 10;  // average of last 10 samples

//Ac -> Line acceleartion; Gy -> Angular velocity 

// Buffers for MPU1 for calculating average
int16_t AcXBuffer1[SMOOTH_WINDOW];
int16_t AcYBuffer1[SMOOTH_WINDOW];
int16_t AcZBuffer1[SMOOTH_WINDOW];
int16_t GyXBuffer1[SMOOTH_WINDOW];
int16_t GyYBuffer1[SMOOTH_WINDOW];
int16_t GyZBuffer1[SMOOTH_WINDOW];
int indexBuf1 = 0;
bool bufferFilled1 = false;

// Buffers for MPU2
int16_t AcXBuffer2[SMOOTH_WINDOW];
int16_t AcYBuffer2[SMOOTH_WINDOW];
int16_t AcZBuffer2[SMOOTH_WINDOW];
int16_t GyXBuffer2[SMOOTH_WINDOW];
int16_t GyYBuffer2[SMOOTH_WINDOW];
int16_t GyZBuffer2[SMOOTH_WINDOW];
int indexBuf2 = 0;
bool bufferFilled2 = false;

// Sensitivity scaling
float gain = 20.0; 

// Need to be calibrated 

double x;
double y;
double z;

//Second MPU6050

double x2;
double y2;
double z2;

/*Autotune flex parameter
  For Debug Mode. Check the upper and lowe limit of the flex sensors
  3 Flex sensors used. Thumb, Middle, Pinkie
*/
int thumb_high = 960;
int thumb_low = 870;
int finger_high = 920;
int finger_low = 837;
int pinkie_high = 950;
int pinkie_low = 890;

//Stop Caliberating the Flex Sensor when complete
bool bool_caliberate = true;

//How often to send values to the Robotic Arm
int response_time = 100;

void setup() {
  
  pinMode(3, OUTPUT);
  Wire.begin();
  Wire.beginTransmission(MPU1);
  Wire.write(0x6B);// PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true); Wire.begin();
  Wire.beginTransmission(MPU2);
  Wire.write(0x6B);// PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  BT.begin(9600);    
  delay(1000);

}
void loop() {

  float AcX1, AcY1, AcZ1;
  float GyX1, GyY1, GyZ1;

  float AcX2, AcY2, AcZ2;
  float GyX2, GyY2, GyZ2;

  /*
    Note: Serial.print() would send all values the robotic arm using via bluetooth.
  */


  digitalWrite(3, HIGH); //Use basic LED as visual indicator if value being sent

  // debug_flex(); //Debug Mode on/off
  // Serial.println("test");

  // Get data from first MPU
  GetMpuValue(MPU1,
              AcX1, AcY1, AcZ1,
              GyX1, GyY1, GyZ1,
              AcXBuffer1, AcYBuffer1, AcZBuffer1,
              GyXBuffer1, GyYBuffer1, GyZBuffer1,
              indexBuf1, bufferFilled1);

  // Convertion
  x = (AcX1 / 16384.0) * gain;
  y = (AcY1 / 16384.0) * gain;
  z = (AcZ1 / 16384.0) * gain;
  delay(10);

  //get values for second mpu having address of 0x69
  GetMpuValue(MPU2,
              AcX2, AcY2, AcZ2,
              GyX2, GyY2, GyZ2,
              AcXBuffer2, AcYBuffer2, AcZBuffer2,
              GyXBuffer2, GyYBuffer2, GyZBuffer2,
              indexBuf2, bufferFilled2);

  x2 = (AcX2 / 16384.0) * gain;
  y2 = (AcY2 / 16384.0) * gain;
  z2 = (AcZ2 / 16384.0) * gain;
  delay(10);


  //Print out a value, based on the change of the XYZ co-ordinates of 1st or 2nd MPU

  //Claw rotates left
  if ( y > 12 && y < 20 ) {
    Serial.println("L");
    BT.println("L");
    delay(response_time);
  }

  //Claw rotates  right
  if (y < -9 && y > -20) {
    Serial.println("R");
    BT.println("R");
    delay(response_time);
  }

  // Wrist Up
  if ( x > 12.5) {
    Serial.println("G");
    BT.println("G");
    delay(response_time);
  }

  // /Wrist Down
  if ( x < -15.72) {
    Serial.println("U");
    BT.println("U");
    delay(response_time);
  }

  //  Shoulder up
  if (x2 >9) {
    Serial.print("C");
    BT.print("C");
    delay(response_time);
  }

  // Shoulder down 
  if (x2 < -12) {
    Serial.print("c");
    BT.print("c");
    delay(response_time);
  }

  // read the values from Flex Sensors to Arduino
  pinkie = analogRead(pinkie_Data);
  finger = analogRead(finger_Data);
  thumb = analogRead(thumb_Data);

  //Calibrate to find upper and lower limit of the Flex Sensor
  if (bool_caliberate == false ) {
    delay(1000);

    thumb_high = (thumb * 1.15);
    thumb_low = (thumb * 0.9);

    finger_high = (finger * 1.03);
    finger_low = (finger * 0.8);

    pinkie_high = (pinkie * 1.06);
    pinkie_low = (pinkie * 0.8);

    bool_caliberate = true;
  }

  delay(response_time);

  // Pinkie
  if (pinkie >= pinkie_high) {
    Serial.print("P");
    BT.print("P");
    delay(response_time);

  }
  if (pinkie <= pinkie_low ) {
    Serial.print("p");
    BT.print("p");
    delay(response_time);
  }


  // thumb 1 - thumb (Base Rotation)
  if (thumb >= thumb_high) {
    Serial.print("T");
    BT.print("T");
    delay(response_time);
  }

  if (thumb <= thumb_low) {
    Serial.print("t");
    BT.print("t");
    delay(response_time);
  }

  // finger 1 - Claw Bend/Open
  if (finger >= finger_high) {
    Serial.print("F");
    BT.print("F");
    delay(response_time);
  }

  if (finger <= finger_low) {
    Serial.print("f");
    BT.print("f");
    delay(response_time);
  }
  else {
    delay(5);
  }
}

// Function: Read and smooth accel + gyro values and take averages
void GetMpuValue(
  const int MPU,
  float &AcX, float &AcY, float &AcZ,
  float &GyX, float &GyY, float &GyZ,
  int16_t *bufX, int16_t *bufY, int16_t *bufZ,
  int16_t *gBufX, int16_t *gBufY, int16_t *gBufZ,
  int &idx, bool &filled
) {
  // Read Accelerometer 
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  int16_t rawX = Wire.read() << 8 | Wire.read();
  int16_t rawY = Wire.read() << 8 | Wire.read();
  int16_t rawZ = Wire.read() << 8 | Wire.read();

  // Read Gyroscope 
  Wire.beginTransmission(MPU);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  int16_t rawGX = Wire.read() << 8 | Wire.read();
  int16_t rawGY = Wire.read() << 8 | Wire.read();
  int16_t rawGZ = Wire.read() << 8 | Wire.read();

  // Store in circular buffers 
  bufX[idx] = rawX;
  bufY[idx] = rawY;
  bufZ[idx] = rawZ;

  gBufX[idx] = rawGX;
  gBufY[idx] = rawGY;
  gBufZ[idx] = rawGZ;

  idx++;
  if (idx >= SMOOTH_WINDOW) {
    idx = 0;
    filled = true;
  }

  // Compute moving average for both accel + gyro 
  long sumX = 0, sumY = 0, sumZ = 0;
  long sumGX = 0, sumGY = 0, sumGZ = 0;
  int count = filled ? SMOOTH_WINDOW : idx;
  for (int i = 0; i < count; i++) {
    sumX += bufX[i];
    sumY += bufY[i];
    sumZ += bufZ[i];

    sumGX += gBufX[i];
    sumGY += gBufY[i];
    sumGZ += gBufZ[i];
  }

  AcX = (float)sumX / count;
  AcY = (float)sumY / count;
  AcZ = (float)sumZ / count;

  GyX = (float)sumGX / count;
  GyY = (float)sumGY / count;
  GyZ = (float)sumGZ / count;
}