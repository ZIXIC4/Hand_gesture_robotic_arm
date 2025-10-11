#include <Wire.h>

// MPU6050 Addresses
const int MPU1 = 0x68;
const int MPU2 = 0x69; 

const int SMOOTH_WINDOW = 10;  // average of last 10 samples

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


void setup() {
  Wire.begin();

  // Initialise MPU1 
  Wire.beginTransmission(MPU1);
  Wire.write(0x6B);
  Wire.write(0);   
  Wire.endTransmission(true);

  // Initialise MPU2
  /*
  Wire.beginTransmission(MPU2);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  */

  Serial.begin(9600);
  delay(1000);

}

void loop() {
  float AcX1, AcY1, AcZ1;
  float GyX1, GyY1, GyZ1;

  // float AcX2, AcY2, AcZ2;
  // float GyX2, GyY2, GyZ2;

  // Get data from first MPU
  GetMpuValue(MPU1,
              AcX1, AcY1, AcZ1,
              GyX1, GyY1, GyZ1,
              AcXBuffer1, AcYBuffer1, AcZBuffer1,
              GyXBuffer1, GyYBuffer1, GyZBuffer1,
              indexBuf1, bufferFilled1);

  // Convertion
  AcX1 = (AcX1 / 16384.0) * gain;
  AcY1 = (AcY1 / 16384.0) * gain;
  AcZ1 = (AcZ1 / 16384.0) * gain;

  GyX1 /= 131.0;
  GyY1 /= 131.0;
  GyZ1 /= 131.0;

  // Serial Plotter Output 
  Serial.print(AcX1); Serial.print("\t");
  Serial.print(AcY1); Serial.print("\t");
  Serial.print(AcZ1); Serial.print("\t");
  Serial.print(GyX1); Serial.print("\t");
  Serial.print(GyY1); Serial.print("\t");
  Serial.println(GyZ1);

  // Second MPU
  /*
  GetMpuValue(MPU2,
              AcX2, AcY2, AcZ2,
              GyX2, GyY2, GyZ2,
              AcXBuffer2, AcYBuffer2, AcZBuffer2,
              GyXBuffer2, GyYBuffer2, GyZBuffer2,
              indexBuf2, bufferFilled2);

  AcX2 = (AcX2 / 16384.0) * gain;
  AcY2 = (AcY2 / 16384.0) * gain;
  AcZ2 = (AcZ2 / 16384.0) * gain;

  GyX2 /= 131.0;
  GyY2 /= 131.0;
  GyZ2 /= 131.0;

  Serial.print(AcX2); Serial.print("\t");
  Serial.print(AcY2); Serial.print("\t");
  Serial.print(AcZ2); Serial.print("\t");
  Serial.print(GyX2); Serial.print("\t");
  Serial.print(GyY2); Serial.print("\t");
  Serial.println(GyZ2);
  */

  delay(20); // ~50Hz sampling
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
