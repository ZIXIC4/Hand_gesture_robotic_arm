#include "HCPCA9685.h"
#include <SoftwareSerial.h>

#define I2CAdd 0x40
#define BT_RX 10   // Arduino RX (connects to BT TX)
#define BT_TX 11   // Arduino TX (connects to BT RX)

HCPCA9685 HCPCA9685(I2CAdd);
SoftwareSerial Bluetooth(BT_RX, BT_TX);  // RX, TX

// Servo channel assignments
#define BASE_STEPPER_ENABLE 9
#define BASE_STEPPER_DIR 4
#define BASE_STEPPER_STEP 5
#define ELBOW_SERVO_L 0
#define ELBOW_SERVO_R 1
#define SHOULDER_SERVO 2
#define WRIST1_SERVO 3
#define WRIST_ROTATE_SERVO 4
#define GRIPPER_SERVO 5

// Servo position limits (adjust based on your physical limits)
const int SHOULDER_MIN = 70;
const int SHOULDER_MAX = 400;
const int SHOULDER_CENTER = 235;

const int ELBOW_MIN = 10;
const int ELBOW_MAX = 180;
const int ELBOW_CENTER = 10;

const int WRIST1_MIN = 10;
const int WRIST1_MAX = 380;
const int WRIST1_CENTER = 195;

const int WRIST_ROTATE_MIN = 10;
const int WRIST_ROTATE_MAX = 380;
const int WRIST_ROTATE_CENTER = 195;

const int GRIPPER_MIN = 10;
const int GRIPPER_MAX = 120;
const int GRIPPER_CENTER = 65;

// IMU and Flex sensor ranges (from your calibration data)
const float IMU1_X_MIN = -20.0;
const float IMU1_X_MAX = 20.0;
const float IMU1_Y_MIN = -20.0;
const float IMU1_Y_MAX = 20.0;

const float IMU2_X_MIN = -15.0;
const float IMU2_X_MAX = 15.0;

const int THUMB_FLEX_MIN = 870;
const int THUMB_FLEX_MAX = 960;
const int FINGER_FLEX_MIN = 837;
const int FINGER_FLEX_MAX = 920;
const int PINKIE_FLEX_MIN = 890;
const int PINKIE_FLEX_MAX = 950;

// Current servo positions
int currentShoulderPos = SHOULDER_CENTER;
int currentElbowPos = ELBOW_CENTER;
int currentWrist1Pos = WRIST1_CENTER;
int currentWristRotatePos = WRIST_ROTATE_CENTER;
int currentGripperPos = GRIPPER_CENTER;

// Smoothing variables
const int SMOOTH_FACTOR = 5; // Adjust for smoothness (higher = smoother but slower response)
int targetShoulderPos = SHOULDER_CENTER;
int targetElbowPos = ELBOW_CENTER;
int targetWrist1Pos = WRIST1_CENTER;
int targetWristRotatePos = WRIST_ROTATE_CENTER;
int targetGripperPos = GRIPPER_CENTER;

// Communication variables
String inputString = "";
bool stringComplete = false;

// Timing
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 50; // Update servos every 50ms

void setup() {
  // Initialize PCA9685
  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);
  
  // Initialize stepper pins
  pinMode(BASE_STEPPER_STEP, OUTPUT);
  pinMode(BASE_STEPPER_DIR, OUTPUT);
  pinMode(BASE_STEPPER_ENABLE, OUTPUT);
  digitalWrite(BASE_STEPPER_ENABLE, HIGH); // Disable stepper initially
  
  // Initialize communication
  Serial.begin(9600);
  Bluetooth.begin(9600);
  
  Serial.println("=== Live Mapping Robotic Arm Ready ===");
  Serial.println("Expected data format: IMU1_X,IMU1_Y,IMU2_X,THUMB,FINGER,PINKIE");
  
  // Move to initial positions
  moveToInitialPosition();
  delay(2000);
}

void loop() {
  // Read incoming data from Bluetooth
  readBluetoothData();
  
  // Process complete data packets
  if (stringComplete) {
    processLiveMappingData(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Update servo positions smoothly
  if (millis() - lastUpdate > UPDATE_INTERVAL) {
    updateServoPositions();
    lastUpdate = millis();
  }
}

void readBluetoothData() {
  while (Bluetooth.available()) {
    char inChar = (char)Bluetooth.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
  
  // Also read from Serial for testing
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void processLiveMappingData(String data) {
  // Expected format: IMU1_X,IMU1_Y,IMU2_X,THUMB,FINGER,PINKIE
  // Parse comma-separated values
  
  int commaIndex = 0;
  int lastCommaIndex = 0;
  float values[6];
  int valueIndex = 0;
  
  // Parse the comma-separated string
  for (int i = 0; i <= data.length(); i++) {
    if (data.charAt(i) == ',' || i == data.length()) {
      if (valueIndex < 6) {
        String valueStr = data.substring(lastCommaIndex, i);
        values[valueIndex] = valueStr.toFloat();
        valueIndex++;
      }
      lastCommaIndex = i + 1;
    }
  }
  
  if (valueIndex == 6) {
    float imu1_x = values[0];
    float imu1_y = values[1];
    float imu2_x = values[2];
    int thumb_flex = (int)values[3];
    int finger_flex = (int)values[4];
    int pinkie_flex = (int)values[5];
    
    // Map IMU and flex data to servo positions
    mapHandToServos(imu1_x, imu1_y, imu2_x, thumb_flex, finger_flex, pinkie_flex);
    
    // Debug output
    Serial.print("Mapped - Shoulder: ");
    Serial.print(targetShoulderPos);
    Serial.print(", Elbow (Locked): ");
    Serial.print(targetElbowPos);
    Serial.print(", Wrist1: ");
    Serial.print(targetWrist1Pos);
    Serial.print(", WristRot: ");
    Serial.print(targetWristRotatePos);
    Serial.print(", Gripper: ");
    Serial.println(targetGripperPos);
  }
}

void mapHandToServos(float imu1_x, float imu1_y, float imu2_x, int thumb_flex, int finger_flex, int pinkie_flex) {
  // Map IMU2 X-axis (upper arm) to shoulder servo (up/down movement)
  targetShoulderPos = mapFloat(imu2_x, IMU2_X_MIN, IMU2_X_MAX, SHOULDER_MIN, SHOULDER_MAX);
  targetShoulderPos = constrain(targetShoulderPos, SHOULDER_MIN, SHOULDER_MAX);
  
  // Map IMU1 X-axis (palm tilt) to wrist servo (up/down movement)
  targetWrist1Pos = mapFloat(imu1_x, IMU1_X_MIN, IMU1_X_MAX, WRIST1_MIN, WRIST1_MAX);
  targetWrist1Pos = constrain(targetWrist1Pos, WRIST1_MIN, WRIST1_MAX);
  
  // Map IMU1 Y-axis (palm rotation) to wrist rotation servo
  targetWristRotatePos = mapFloat(imu1_y, IMU1_Y_MIN, IMU1_Y_MAX, WRIST_ROTATE_MIN, WRIST_ROTATE_MAX);
  targetWristRotatePos = constrain(targetWristRotatePos, WRIST_ROTATE_MIN, WRIST_ROTATE_MAX);
  
  // Elbow joint disabled — keep fixed at 10° (ELBOW_MIN)
  targetElbowPos = ELBOW_MIN;

  
  // Map finger flex sensor to gripper (finger controls grip strength)
  targetGripperPos = mapFloat(finger_flex, FINGER_FLEX_MIN, FINGER_FLEX_MAX, GRIPPER_MAX, GRIPPER_MIN); // Inverted
  targetGripperPos = constrain(targetGripperPos, GRIPPER_MIN, GRIPPER_MAX);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void updateServoPositions() {
  // Smooth movement towards target positions
  currentShoulderPos += (targetShoulderPos - currentShoulderPos) / SMOOTH_FACTOR;
  currentElbowPos += (targetElbowPos - currentElbowPos) / SMOOTH_FACTOR;
  currentWrist1Pos += (targetWrist1Pos - currentWrist1Pos) / SMOOTH_FACTOR;
  currentWristRotatePos += (targetWristRotatePos - currentWristRotatePos) / SMOOTH_FACTOR;
  currentGripperPos += (targetGripperPos - currentGripperPos) / SMOOTH_FACTOR;
  
  // Update servos with current positions
  HCPCA9685.Servo(SHOULDER_SERVO, currentShoulderPos);
  
  // Elbow servos (dual servo setup)
  HCPCA9685.Servo(ELBOW_SERVO_L, (ELBOW_MAX - currentElbowPos) - 10);
  HCPCA9685.Servo(ELBOW_SERVO_R, currentElbowPos);
  
  HCPCA9685.Servo(WRIST1_SERVO, currentWrist1Pos);
  HCPCA9685.Servo(WRIST_ROTATE_SERVO, currentWristRotatePos);
  HCPCA9685.Servo(GRIPPER_SERVO, currentGripperPos);
}

void moveToInitialPosition() {
  Serial.println("Moving to initial position...");
  
  // Set target positions to center
  targetShoulderPos = SHOULDER_CENTER;
  targetElbowPos = ELBOW_CENTER;
  targetWrist1Pos = WRIST1_CENTER;
  targetWristRotatePos = WRIST_ROTATE_CENTER;
  targetGripperPos = GRIPPER_CENTER;
  
  // Move gradually to initial position
  for (int i = 0; i < 50; i++) {
    updateServoPositions();
    delay(50);
  }
  
  Serial.println("Initial position reached.");
}

// Emergency stop function (can be called if needed)
void emergencyStop() {
  // Stop all servos at current position
  Serial.println("EMERGENCY STOP!");
  // Disable stepper
  digitalWrite(BASE_STEPPER_ENABLE, HIGH);
}

// Manual control functions (for testing/calibration)
void testServos() {
  Serial.println("Testing servos...");
  
  // Test each servo individually
  for (int pos = SHOULDER_CENTER; pos <= SHOULDER_CENTER + 50; pos += 10) {
    HCPCA9685.Servo(SHOULDER_SERVO, pos);
    delay(500);
  }
  
  for (int pos = ELBOW_CENTER; pos <= ELBOW_CENTER + 30; pos += 10) {
    HCPCA9685.Servo(ELBOW_SERVO_L, (ELBOW_MAX - pos) - 10);
    HCPCA9685.Servo(ELBOW_SERVO_R, pos);
    delay(500);
  }
  
  // Return to center
  moveToInitialPosition();
}