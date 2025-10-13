#include <Wire.h>

// === MPU6050 I2C Addresses ===
const int MPU1 = 0x68;  // AD0 -> GND
const int MPU2 = 0x69;  // AD0 -> 3.3V

// === Moving average window size ===
const int SMOOTH_WINDOW = 10;

// === Buffers for MPU1 ===
int16_t AcXBuffer1[SMOOTH_WINDOW];
int16_t AcYBuffer1[SMOOTH_WINDOW];
int16_t AcZBuffer1[SMOOTH_WINDOW];
int16_t GyXBuffer1[SMOOTH_WINDOW];
int16_t GyYBuffer1[SMOOTH_WINDOW];
int16_t GyZBuffer1[SMOOTH_WINDOW];
int indexBuf1 = 0;
bool bufferFilled1 = false;

// === Buffers for MPU2 ===
int16_t AcXBuffer2[SMOOTH_WINDOW];
int16_t AcYBuffer2[SMOOTH_WINDOW];
int16_t AcZBuffer2[SMOOTH_WINDOW];
int16_t GyXBuffer2[SMOOTH_WINDOW];
int16_t GyYBuffer2[SMOOTH_WINDOW];
int16_t GyZBuffer2[SMOOTH_WINDOW];
int indexBuf2 = 0;
bool bufferFilled2 = false;

// === Scaling gain ===
float gain = 20.0; // amplify motion for servo control

// Timer for header reprint
unsigned long lastHeaderTime = 0;

void setup() {
  Wire.begin();
  Wire.setClock(100000); // standard I2C speed (safe for dual MPU)

  // Initialize MPU1
  Wire.beginTransmission(MPU1);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Initialize MPU2
  Wire.beginTransmission(MPU2);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.begin(9600);
  delay(1000);

  // Print header for Serial Plotter
  printHeader();
}

void loop() {
  float AcX1, AcY1, AcZ1, GyX1, GyY1, GyZ1;
  float AcX2, AcY2, AcZ2, GyX2, GyY2, GyZ2;

  // Read both sensors
  GetMpuValue(MPU1,
              AcX1, AcY1, AcZ1,
              GyX1, GyY1, GyZ1,
              AcXBuffer1, AcYBuffer1, AcZBuffer1,
              GyXBuffer1, GyYBuffer1, GyZBuffer1,
              indexBuf1, bufferFilled1);

  GetMpuValue(MPU2,
              AcX2, AcY2, AcZ2,
              GyX2, GyY2, GyZ2,
              AcXBuffer2, AcYBuffer2, AcZBuffer2,
              GyXBuffer2, GyYBuffer2, GyZBuffer2,
              indexBuf2, bufferFilled2);

  // Convert accelerometer readings to g-units
  AcX1 = (AcX1 / 16384.0) * gain;
  AcY1 = (AcY1 / 16384.0) * gain;
  AcZ1 = (AcZ1 / 16384.0) * gain;

  AcX2 = (AcX2 / 16384.0) * gain;
  AcY2 = (AcY2 / 16384.0) * gain;
  AcZ2 = (AcZ2 / 16384.0) * gain;

  // Convert gyro readings to degrees/sec
  GyX1 /= 131.0;
  GyY1 /= 131.0;
  GyZ1 /= 131.0;

  GyX2 /= 131.0;
  GyY2 /= 131.0;
  GyZ2 /= 131.0;

  // --- Output all 12 values in one clean line ---
  Serial.print(AcX1); Serial.print("\t");
  Serial.print(AcY1); Serial.print("\t");
  Serial.print(AcZ1); Serial.print("\t");
  Serial.print(GyX1); Serial.print("\t");
  Serial.print(GyY1); Serial.print("\t");
  Serial.print(GyZ1); Serial.print("\t");
  Serial.print(AcX2); Serial.print("\t");
  Serial.print(AcY2); Serial.print("\t");
  Serial.print(AcZ2); Serial.print("\t");
  Serial.print(GyX2); Serial.print("\t");
  Serial.print(GyY2); Serial.print("\t");
  Serial.println(GyZ2);

  Serial.flush(); // ensure complete line before next update

  // Reprint header every 5 seconds (helps if plotter is reopened)
  if (millis() - lastHeaderTime > 5000) {
    printHeader();
    lastHeaderTime = millis();
  }

  delay(20); // ~50Hz refresh
}

// --- Helper: Print the 12-column header line ---
void printHeader() {
  Serial.println("AcX1\tAcY1\tAcZ1\tGyX1\tGyY1\tGyZ1\tAcX2\tAcY2\tAcZ2\tGyX2\tGyY2\tGyZ2");
  Serial.flush();
}

// --- Helper: Read + Smooth Accel & Gyro Data ---
void GetMpuValue(
  const int MPU,
  float &AcX, float &AcY, float &AcZ,
  float &GyX, float &GyY, float &GyZ,
  int16_t *bufX, int16_t *bufY, int16_t *bufZ,
  int16_t *gBufX, int16_t *gBufY, int16_t *gBufZ,
  int &idx, bool &filled
) {
  // --- Accelerometer ---
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  int16_t rawX = Wire.read() << 8 | Wire.read();
  int16_t rawY = Wire.read() << 8 | Wire.read();
  int16_t rawZ = Wire.read() << 8 | Wire.read();

  // --- Gyroscope ---
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  int16_t rawGX = Wire.read() << 8 | Wire.read();
  int16_t rawGY = Wire.read() << 8 | Wire.read();
  int16_t rawGZ = Wire.read() << 8 | Wire.read();

  // Store in circular buffers
  bufX[idx] = rawX; bufY[idx] = rawY; bufZ[idx] = rawZ;
  gBufX[idx] = rawGX; gBufY[idx] = rawGY; gBufZ[idx] = rawGZ;

  idx++;
  if (idx >= SMOOTH_WINDOW) {
    idx = 0;
    filled = true;
  }

  // Compute moving averages
  long sumX=0, sumY=0, sumZ=0;
  long sumGX=0, sumGY=0, sumGZ=0;
  int count = filled ? SMOOTH_WINDOW : idx;

  for (int i=0; i<count; i++) {
    sumX  += bufX[i];
    sumY  += bufY[i];
    sumZ  += bufZ[i];
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
