#include<Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial BT(11, 10); // nano's RX, TX

//Create flex Sensors
int pinkie = 0; //Pinkie finger
int finger = 0; //Middle finger 
int thumb = 0; //Thumb

int pinkie_Data = A1;
int finger_Data = A2;
int thumb_Data = A3;

// MPU1 -> Palm movement； MPU2 -> Upper arm movement
const int MPU2 = 0x69, MPU1 = 0x68;
const int SMOOTH_WINDOW = 5;  // Smaller window for more responsive live mapping

// Buffers for MPU1 (Palm)
int16_t AcXBuffer1[SMOOTH_WINDOW];
int16_t AcYBuffer1[SMOOTH_WINDOW];
int16_t AcZBuffer1[SMOOTH_WINDOW];
int indexBuf1 = 0;
bool bufferFilled1 = false;

// Buffers for MPU2 (Upper arm)
int16_t AcXBuffer2[SMOOTH_WINDOW];
int16_t AcYBuffer2[SMOOTH_WINDOW]; 
int16_t AcZBuffer2[SMOOTH_WINDOW];
int indexBuf2 = 0;
bool bufferFilled2 = false;

// Sensitivity scaling
float gain = 20.0; 

// Live sensor values
double palm_x = 0, palm_y = 0, palm_z = 0;
double arm_x = 0, arm_y = 0, arm_z = 0;

// Flex sensor calibrated ranges
int thumb_high = 960;
int thumb_low = 870;
int finger_high = 920;
int finger_low = 837;
int pinkie_high = 950;
int pinkie_low = 890;

// Calibration mode
bool calibration_mode = false;
bool calibration_complete = false;
unsigned long calibration_start = 0;
const unsigned long CALIBRATION_DURATION = 10000; // 10 seconds

// Data transmission
unsigned long last_transmission = 0;
const unsigned long TRANSMISSION_INTERVAL = 50; // Send data every 50ms (20 Hz)

void setup() {
  pinMode(3, OUTPUT);  // Status LED
  
  // Initialize I2C and MPUs
  Wire.begin();
  
  // Initialize MPU1 (Palm)
  Wire.beginTransmission(MPU1);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up MPU
  Wire.endTransmission(true);
  
  // Initialize MPU2 (Upper arm)
  Wire.beginTransmission(MPU2);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up MPU
  Wire.endTransmission(true);
  
  Serial.begin(9600);
  BT.begin(9600);
  
  Serial.println("=== Live Mapping Glove Initialized ===");
  Serial.println("Send 'c' to start calibration");
  Serial.println("Data format: IMU1_X,IMU1_Y,IMU2_X,THUMB,FINGER,PINKIE");
  
  delay(1000);
}

void loop() {
  // Check for calibration command
  if (Serial.available() > 0 || BT.available() > 0) {
    char command = 0;
    if (Serial.available() > 0) command = Serial.read();
    else if (BT.available() > 0) command = BT.read();
    
    if (command == 'c' && !calibration_mode) {
      startCalibration();
    }
  }
  
  // Handle calibration mode
  if (calibration_mode) {
    handleCalibration();
    return; // Skip normal operation during calibration
  }
  
  // Read sensor data
  readSensorData();
  
  // Transmit data at regular intervals
  if (millis() - last_transmission > TRANSMISSION_INTERVAL) {
    transmitLiveData();
    last_transmission = millis();
  }
  
  // Visual indicator
  digitalWrite(3, (millis() / 500) % 2); // Blink LED every 500ms
}

void readSensorData() {
  float AcX1, AcY1, AcZ1;
  float AcX2, AcY2, AcZ2;
  
  // Get data from Palm MPU (MPU1)
  GetMpuValue(MPU1, AcX1, AcY1, AcZ1, 
              AcXBuffer1, AcYBuffer1, AcZBuffer1,
              indexBuf1, bufferFilled1);
  
  // Convert and scale palm data
  palm_x = (AcX1 / 16384.0) * gain;
  palm_y = (AcY1 / 16384.0) * gain;
  palm_z = (AcZ1 / 16384.0) * gain;
  
  // Get data from Upper Arm MPU (MPU2)
  GetMpuValue(MPU2, AcX2, AcY2, AcZ2,
              AcXBuffer2, AcYBuffer2, AcZBuffer2,
              indexBuf2, bufferFilled2);
  
  // Convert and scale arm data
  arm_x = (AcX2 / 16384.0) * gain;
  arm_y = (AcY2 / 16384.0) * gain;
  arm_z = (AcZ2 / 16384.0) * gain;
  
  // Read flex sensors
  thumb = analogRead(thumb_Data);
  finger = analogRead(finger_Data);
  //pinkie = analogRead(pinkie_Data);
}

void transmitLiveData() {
  // Format: IMU1_X,IMU1_Y,IMU2_X,THUMB,FINGER,PINKIE
  String dataPacket = String(palm_x, 2) + "," + 
                     String(palm_y, 2) + "," + 
                     String(arm_x, 2) + "," + 
                     String(thumb) + "," + 
                     String(finger) + "," + 
                     "0";
  
  // Send via Bluetooth
  BT.println(dataPacket);
  
  // Debug output to Serial (comment out for performance if needed)
  Serial.println("TX: " + dataPacket);
}

void startCalibration() {
  calibration_mode = true;
  calibration_start = millis();
  
  Serial.println("=== CALIBRATION MODE ===");
  Serial.println("Move your hand through full range of motion for 10 seconds...");
  BT.println("CALIBRATION_START");
  
  // Reset calibration values
  thumb_high = 0; thumb_low = 1023;
  finger_high = 0; finger_low = 1023;  
  pinkie_high = 0; pinkie_low = 1023;
}

void handleCalibration() {
  // Read current sensor values
  int current_thumb = analogRead(thumb_Data);
  int current_finger = analogRead(finger_Data);
  int current_pinkie = analogRead(pinkie_Data);
  
  // Update min/max values
  if (current_thumb > thumb_high) thumb_high = current_thumb;
  if (current_thumb < thumb_low) thumb_low = current_thumb;
  
  if (current_finger > finger_high) finger_high = current_finger;
  if (current_finger < finger_low) finger_low = current_finger;
  
  if (current_pinkie > pinkie_high) pinkie_high = current_pinkie;
  if (current_pinkie < pinkie_low) pinkie_low = current_pinkie;
  
  // Show progress
  if ((millis() - calibration_start) % 1000 < 50) { // Print once per second
    unsigned long remaining = CALIBRATION_DURATION - (millis() - calibration_start);
    Serial.println("Calibrating... " + String(remaining / 1000) + "s remaining");
  }
  
  // Check if calibration is complete
  if (millis() - calibration_start > CALIBRATION_DURATION) {
    finishCalibration();
  }
}

void finishCalibration() {
  calibration_mode = false;
  calibration_complete = true;
  
  Serial.println("=== CALIBRATION COMPLETE ===");
  Serial.println("Thumb range: " + String(thumb_low) + " - " + String(thumb_high));
  Serial.println("Finger range: " + String(finger_low) + " - " + String(finger_high));
  Serial.println("Pinkie range: " + String(pinkie_low) + " - " + String(pinkie_high));
  
  BT.println("CALIBRATION_COMPLETE");
  
  // Add some margin to the ranges (5%)
  int thumb_margin = (thumb_high - thumb_low) * 0.05;
  int finger_margin = (finger_high - finger_low) * 0.05;
  int pinkie_margin = (pinkie_high - pinkie_low) * 0.05;
  
  thumb_low += thumb_margin;
  thumb_high -= thumb_margin;
  finger_low += finger_margin;
  finger_high -= finger_margin;
  pinkie_low += pinkie_margin;
  pinkie_high -= pinkie_margin;
  
  Serial.println("Starting live mapping mode...");
}

// Function to read and smooth accelerometer values
void GetMpuValue(
  const int MPU,
  float &AcX, float &AcY, float &AcZ,
  int16_t *bufX, int16_t *bufY, int16_t *bufZ,
  int &idx, bool &filled
) {
  // Read Accelerometer data
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Starting register for accel data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  
  int16_t rawX = Wire.read() << 8 | Wire.read();
  int16_t rawY = Wire.read() << 8 | Wire.read();
  int16_t rawZ = Wire.read() << 8 | Wire.read();

  // Store in circular buffer
  bufX[idx] = rawX;
  bufY[idx] = rawY;
  bufZ[idx] = rawZ;

  idx++;
  if (idx >= SMOOTH_WINDOW) {
    idx = 0;
    filled = true;
  }

  // Compute moving average
  long sumX = 0, sumY = 0, sumZ = 0;
  int count = filled ? SMOOTH_WINDOW : idx;
  
  for (int i = 0; i < count; i++) {
    sumX += bufX[i];
    sumY += bufY[i];
    sumZ += bufZ[i];
  }

  AcX = (float)sumX / count;
  AcY = (float)sumY / count;
  AcZ = (float)sumZ / count;
}

// Debug function to print current sensor readings
void debugSensors() {
  Serial.println("=== SENSOR DEBUG ===");
  Serial.println("Palm IMU - X:" + String(palm_x, 2) + " Y:" + String(palm_y, 2) + " Z:" + String(palm_z, 2));
  Serial.println("Arm IMU - X:" + String(arm_x, 2) + " Y:" + String(arm_y, 2) + " Z:" + String(arm_z, 2));
  Serial.println("Flex - Thumb:" + String(thumb) + " Finger:" + String(finger) + " Pinkie:" + String(pinkie));
  Serial.println("==================");
}