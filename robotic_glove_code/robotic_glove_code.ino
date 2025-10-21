#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial BT(11, 10); // nano's RX, TX

// Create thumb Sensors
int pinkie = 0; // Pinkie thumb
int finger = 0; // finger thumb
int thumb  = 0; // Index thumb

int pinkie_Data = A1;
int finger_Data = A2;
int thumb_Data  = A3;

// MPU1 -> Palm movement； MPU2 -> Upper arm movement
const int MPU2 = 0x69, MPU1 = 0x68;
const int SMOOTH_WINDOW = 3;  // average of last 10 samples

// === Flex sensor smoothing ===
const int FLEX_WINDOW = 3;

int  thumbBuf[FLEX_WINDOW]  = {0};
int  fingerBuf[FLEX_WINDOW] = {0};
int  pinkieBuf[FLEX_WINDOW] = {0};

long thumbSum  = 0;
long fingerSum = 0;
long pinkieSum = 0;

int  thumbIdx  = 0,  fingerIdx  = 0,  pinkieIdx  = 0;
bool thumbFull = false, fingerFull = false, pinkieFull = false;

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
double x, y, z;
double x2, y2, z2;

/* Autotune flex parameter
  For Debug Mode. Check the upper and lower limit of the flex sensors
  3 Flex sensors used. Thumb, Middle, Pinkie
*/
int thumb_high  = 915;
int thumb_low   = 852;
int finger_high = 915;
int finger_low  = 840;
int pinkie_high = 950;
int pinkie_low  = 890;

// Stop Calibrating the Flex Sensor when complete
bool bool_caliberate = true;

// How often to send values to the Robotic Arm
int response_time = 500;

// --------- Prototypes ----------
void GetMpuValue(
  const int MPU,
  float &AcX, float &AcY, float &AcZ,
  float &GyX, float &GyY, float &GyZ,
  int16_t *bufX, int16_t *bufY, int16_t *bufZ,
  int16_t *gBufX, int16_t *gBufY, int16_t *gBufZ,
  int &idx, bool &filled
);

int updateMovingAverage(int newVal,
                        int *buf, long &sum,
                        int &idx, bool &filled,
                        const int WIN);
// -------------------------------

void setup() {
  pinMode(3, OUTPUT);
  Wire.begin();

  // Wake MPU1
  Wire.beginTransmission(MPU1);
  Wire.write(0x6B);           // PWR_MGMT_1 register
  Wire.write(0);              // set to zero (wake up)
  Wire.endTransmission(true);

  // Wake MPU2
  Wire.beginTransmission(MPU2);
  Wire.write(0x6B);
  Wire.write(0);
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

  digitalWrite(3, HIGH); // LED as visual indicator if value being sent

  // Get data from first MPU
  GetMpuValue(MPU1,
              AcX1, AcY1, AcZ1,
              GyX1, GyY1, GyZ1,
              AcXBuffer1, AcYBuffer1, AcZBuffer1,
              GyXBuffer1, GyYBuffer1, GyZBuffer1,
              indexBuf1, bufferFilled1);

  // Convert accel (LSB to g) then scale
  x = (AcX1 / 16384.0) * gain;
  y = (AcY1 / 16384.0) * gain;
  z = (AcZ1 / 16384.0) * gain;
  delay(10);

  // Get data from second MPU (0x69)
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

  
  // ----- Motion Commands -----
  // Claw rotates left
  if (y > 8 && y < 25) {
    Serial.println("L");
    BT.println("L");
    delay(response_time);
  }

  // Claw rotates right
  if (y < -12 && y > -20) {
    Serial.println("R");
    BT.println("R");
    delay(response_time);
  }

  // Wrist Up
  if (x > 10) {
    Serial.println("G");
    BT.println("G");
    delay(response_time);
  }

  // Wrist Down
  if (x < -10) {
    Serial.println("U");
    BT.println("U");
    delay(response_time);
  }

  // Shoulder up
  if (x2 < -4 ) {
    Serial.print("C");
    BT.print("C");
    delay(response_time);
  }

  // Shoulder down 
  if (x2 > 10) {
    Serial.print("c");
    BT.print("c");
    delay(response_time);
  }

  

  // ----- Flex Sensors (smoothed to 10-sample moving average) -----
  int thumbRaw  = analogRead(thumb_Data);
  int fingerRaw = analogRead(finger_Data);
  int pinkieRaw = analogRead(pinkie_Data);

  // Smooth and overwrite live vars
  thumb  = updateMovingAverage(thumbRaw,  thumbBuf,  thumbSum,  thumbIdx,  thumbFull,  FLEX_WINDOW);
  finger = updateMovingAverage(fingerRaw, fingerBuf, fingerSum, fingerIdx, fingerFull, FLEX_WINDOW);
  pinkie = updateMovingAverage(pinkieRaw, pinkieBuf, pinkieSum, pinkieIdx, pinkieFull, FLEX_WINDOW);

  // ----- Optional: Calibrate using the averaged readings -----
  if (bool_caliberate == false) {
    delay(1000);

    thumb_high  = (thumb  * 1.15);
    thumb_low   = (thumb  * 0.90);

    finger_high = (finger * 1.03);
    finger_low  = (finger * 0.80);

    pinkie_high = (pinkie * 1.06);
    pinkie_low  = (pinkie * 0.80);

    bool_caliberate = true;
  }

  // ----- Threshold-based commands using averaged values -----
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

  // Thumb (Base Rotation)
  if (thumb >= thumb_high) {
    Serial.print("S");
    BT.print("S");
    delay(response_time);
  }
  if (thumb <= thumb_low) {
    Serial.print("O");
    BT.print("O");
    delay(response_time);
  }

  // Finger (Claw Bend/Open)
  if (finger >= finger_high) {
    Serial.print("F");
    BT.print("F");
    delay(response_time);
  }
  if (finger <= finger_low) {
    Serial.print("f");
    BT.print("f");
    delay(response_time);
  } else {
    delay(5);
  }
  
}


// -------- Functions --------

// Read and smooth accel + gyro values and take averages
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
  bufX[idx]  = rawX;
  bufY[idx]  = rawY;
  bufZ[idx]  = rawZ;

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
  if (count == 0) count = 1;

  for (int i = 0; i < count; i++) {
    sumX  += bufX[i];
    sumY  += bufY[i];
    sumZ  += bufZ[i];

    sumGX += gBufX[i];
    sumGY += gBufY[i];
    sumGZ += gBufZ[i];
  }

  AcX = (float)sumX  / count;
  AcY = (float)sumY  / count;
  AcZ = (float)sumZ  / count;

  GyX = (float)sumGX / count;
  GyY = (float)sumGY / count;
  GyZ = (float)sumGZ / count;
}

// Update circular buffer + running sum; return current moving average
int updateMovingAverage(int newVal,
                        int *buf, long &sum,
                        int &idx, bool &filled,
                        const int WIN) {
  if (filled) {
    sum -= buf[idx];          // remove oldest
  }
  buf[idx] = newVal;          // add newest
  sum += newVal;

  idx++;
  if (idx >= WIN) {           // wrap and mark filled after first full pass
    idx = 0;
    filled = true;
  }

  int count = filled ? WIN : idx;  // how many valid samples we have
  if (count == 0) count = 1;       // safety
  return (int)(sum / count);
}