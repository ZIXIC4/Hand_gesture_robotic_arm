#include "HCPCA9685.h"
#include <SoftwareSerial.h>

#define I2CAdd 0x40
#define BT_RX 10   // Arduino RX (connects to BT TX)
#define BT_TX 11   // Arduino TX (connects to BT RX)

HCPCA9685 HCPCA9685(I2CAdd);
SoftwareSerial Bluetooth(BT_RX, BT_TX);  // RX, TX

// Initial parking positions
const int servo_joint_L_parking_pos = 60;
const int servo_joint_R_parking_pos = 60;
const int servo_joint_1_parking_pos = 70;
const int servo_joint_2_parking_pos = 47;
const int servo_joint_3_parking_pos = 63;
const int servo_joint_4_parking_pos = 63;

// Step increments
int servo_joint_L_pos_increment = 20;
int servo_joint_R_pos_increment = 20;
int servo_joint_1_pos_increment = 20;
int servo_joint_2_pos_increment = 50;
int servo_joint_3_pos_increment = 60;
int servo_joint_4_pos_increment = 40;

// Position trackers
int servo_joint_L_parking_pos_i = servo_joint_L_parking_pos;
int servo_joint_R_parking_pos_i = servo_joint_R_parking_pos;
int servo_joint_1_parking_pos_i = servo_joint_1_parking_pos;
int servo_joint_2_parking_pos_i = servo_joint_2_parking_pos;
int servo_joint_3_parking_pos_i = servo_joint_3_parking_pos;
int servo_joint_4_parking_pos_i = servo_joint_4_parking_pos;

// Min/max limits
int servo_joint_L_min_pos = 10;
int servo_joint_L_max_pos = 180;
int servo_joint_R_min_pos = 10;
int servo_joint_R_max_pos = 180;
int servo_joint_1_min_pos = 10;
int servo_joint_1_max_pos = 400;
int servo_joint_2_min_pos = 10;
int servo_joint_2_max_pos = 380;
int servo_joint_3_min_pos = 10;
int servo_joint_3_max_pos = 380;
int servo_joint_4_min_pos = 10;
int servo_joint_4_max_pos = 120;

char state = 0;
int response_time = 5;
int response_time_4 = 2;
int loop_check = 0;
int response_time_fast = 20;
int action_delay = 600;

// Stepper definitions
const int dirPin = 4;
const int stepPin = 5;
const int Enable = 9;
const int stepsPerRevolution = 300;
int stepDelay = 1300;
const int stepsPerRevolutionSmall = 60;
int stepDelaySmall = 9500;

unsigned int Pos;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(Enable, OUTPUT);

  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);

  Serial.begin(9600);
  Bluetooth.begin(9600);
  Serial.println("=== Bluetooth <-> Serial Bridge Ready ===");
  delay(2000);

  wakeUp();
}

void loop() {
  if (Bluetooth.available() > 0 || Serial.available() > 0) {
    if (Bluetooth.available() > 0)
      state = Bluetooth.read();
    else if (Serial.available() > 0)
      state = Serial.read();

    Serial.print("Command: ");
    Serial.println(state);

    switch (state) {
      case 'S': baseRotateLeft(); break;
      case 'O': baseRotateRight(); break;
      case 'c': shoulderServoForward(); break;
      case 'C': shoulderServoBackward(); break;
      case 'p': elbowServoForward(); break;
      case 'P': elbowServoBackward(); break;
      case 'U': wristServo1Backward(); break;
      case 'G': wristServo1Forward(); break;
      case 'R': wristServoCW(); break;
      case 'L': wristServoCCW(); break;
      case 'F': gripperServoBackward(); break;
      case 'f': gripperServoForward(); break;
      default: break;
    }
  }
}

/* === UPDATED SERVO FUNCTIONS (Immediate Increment) === */

void gripperServoForward() {
  if (servo_joint_4_parking_pos_i > servo_joint_4_min_pos) {
    servo_joint_4_parking_pos_i -= servo_joint_4_pos_increment;
    HCPCA9685.Servo(5, servo_joint_4_parking_pos_i);
    delay(response_time);
    Serial.print("Gripper Forward -> ");
    Serial.println(servo_joint_4_parking_pos_i);
  }
}

void gripperServoBackward() {
  if (servo_joint_4_parking_pos_i < servo_joint_4_max_pos) {
    servo_joint_4_parking_pos_i += servo_joint_4_pos_increment;
    HCPCA9685.Servo(5, servo_joint_4_parking_pos_i);
    delay(response_time);
    Serial.print("Gripper Backward -> ");
    Serial.println(servo_joint_4_parking_pos_i);
  }
}

void wristServoCW() {
  if (servo_joint_3_parking_pos_i > servo_joint_3_min_pos) {
    servo_joint_3_parking_pos_i -= servo_joint_3_pos_increment;
    HCPCA9685.Servo(4, servo_joint_3_parking_pos_i);
    delay(response_time_4);
    Serial.print("Wrist CW -> ");
    Serial.println(servo_joint_3_parking_pos_i);
  }
}

void wristServoCCW() {
  if (servo_joint_3_parking_pos_i < servo_joint_3_max_pos) {
    servo_joint_3_parking_pos_i += servo_joint_3_pos_increment;
    HCPCA9685.Servo(4, servo_joint_3_parking_pos_i);
    delay(response_time_4);
    Serial.print("Wrist CCW -> ");
    Serial.println(servo_joint_3_parking_pos_i);
  }
}

void wristServo1Forward() {
  if (servo_joint_2_parking_pos_i < servo_joint_2_max_pos) {
    servo_joint_2_parking_pos_i += servo_joint_2_pos_increment;
    HCPCA9685.Servo(3, servo_joint_2_parking_pos_i);
    delay(response_time);
    Serial.print("Wrist1 Forward -> ");
    Serial.println(servo_joint_2_parking_pos_i);
  }
}

void wristServo1Backward() {
  if (servo_joint_2_parking_pos_i > servo_joint_2_min_pos) {
    servo_joint_2_parking_pos_i -= servo_joint_2_pos_increment;
    HCPCA9685.Servo(3, servo_joint_2_parking_pos_i);
    delay(response_time);
    Serial.print("Wrist1 Backward -> ");
    Serial.println(servo_joint_2_parking_pos_i);
  }
}

void elbowServoForward() {
  if (servo_joint_L_parking_pos_i < servo_joint_L_max_pos) {
    servo_joint_L_parking_pos_i += servo_joint_L_pos_increment;
    HCPCA9685.Servo(0, (servo_joint_L_max_pos - servo_joint_L_parking_pos_i) - 10);
    HCPCA9685.Servo(1, servo_joint_L_parking_pos_i);
    delay(response_time);
    Serial.print("Elbow Down -> ");
    Serial.println(servo_joint_L_parking_pos_i);
  }
}

void elbowServoBackward() {
  if (servo_joint_L_parking_pos_i > servo_joint_L_min_pos) {
    servo_joint_L_parking_pos_i -= servo_joint_L_pos_increment;
    HCPCA9685.Servo(0, servo_joint_L_parking_pos_i - 10);
    HCPCA9685.Servo(1, (servo_joint_L_max_pos - servo_joint_L_parking_pos_i));
    delay(response_time);
    Serial.print("Elbow Up -> ");
    Serial.println(servo_joint_L_parking_pos_i);
  }
}

void shoulderServoForward() {
  if (servo_joint_1_parking_pos_i < servo_joint_1_max_pos) {
    servo_joint_1_parking_pos_i += servo_joint_1_pos_increment;
    HCPCA9685.Servo(2, servo_joint_1_parking_pos_i);
    delay(response_time);
    Serial.print("Shoulder Down -> ");
    Serial.println(servo_joint_1_parking_pos_i);
  }
}

void shoulderServoBackward() {
  if (servo_joint_1_parking_pos_i > servo_joint_1_min_pos) {
    servo_joint_1_parking_pos_i -= servo_joint_1_pos_increment;
    HCPCA9685.Servo(2, servo_joint_1_parking_pos_i);
    delay(response_time);
    Serial.print("Shoulder Up -> ");
    Serial.println(servo_joint_1_parking_pos_i);
  }
}

/* === STEPPER AND WAKE FUNCTIONS (unchanged) === */

void baseRotateLeft() {
  digitalWrite(Enable, LOW);
  digitalWrite(dirPin, HIGH);
  for (int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
  digitalWrite(Enable, HIGH);
  delay(response_time);
}

void baseRotateRight() {
  digitalWrite(Enable, LOW);
  digitalWrite(dirPin, LOW);
  for (int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
  digitalWrite(Enable, HIGH);
  delay(response_time);
}

void wakeUp() {
  if (loop_check == 0) {
    for (Pos = 0; Pos < 10; Pos++) {
      HCPCA9685.Servo(1, Pos);
      delay(response_time_fast);
    }
    for (Pos = 400; Pos > 390; Pos--) {
      HCPCA9685.Servo(2, Pos);
      delay(response_time_fast);
    }
    for (Pos = 10; Pos < 20; Pos++) {
      HCPCA9685.Servo(3, Pos);
      delay(response_time);
    }
    for (Pos = 380; Pos > 50; Pos--) {
      HCPCA9685.Servo(4, Pos);
      delay(response_time);
    }
    for (Pos = 50; Pos < 150; Pos++) {
      HCPCA9685.Servo(4, Pos);
      delay(response_time);
    }
    for (Pos = 19; Pos < 100; Pos++) {
      HCPCA9685.Servo(3, Pos);
      delay(response_time);
    }
    loop_check = 0;
  }
}
