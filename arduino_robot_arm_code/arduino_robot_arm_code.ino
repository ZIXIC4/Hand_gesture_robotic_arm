#include <HCPCA9685.h>
#include <SoftwareSerial.h>

#define I2CAdd 0x40
#define BT_RX 10   // Arduino RX (connects to BT TX)
#define BT_TX 11   // Arduino TX (connects to BT RX)

HCPCA9685 HCPCA9685(I2CAdd);
SoftwareSerial Bluetooth(BT_RX, BT_TX);  // RX, TX

// Initial parking positions
const int servo_joint_L_parking_pos = 10;
const int servo_joint_R_parking_pos = 10;
const int servo_joint_1_parking_pos = 400;
const int servo_joint_2_parking_pos = 297;
const int servo_joint_3_parking_pos = 123;
const int servo_joint_4_parking_pos = 63;

// Step increments
int servo_joint_L_pos_increment = 20;
int servo_joint_R_pos_increment = 20;
int servo_joint_1_pos_increment = 20;
int servo_joint_2_pos_increment = 50;
int servo_joint_3_pos_increment = 60;
int servo_joint_4_pos_increment = 20;

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
int servo_joint_1_min_pos = 180;
int servo_joint_1_max_pos = 400;
int servo_joint_2_min_pos = 197;
int servo_joint_2_max_pos = 440;
int servo_joint_3_min_pos = 10;
int servo_joint_3_max_pos = 380;
int servo_joint_4_min_pos = 10;
int servo_joint_4_max_pos = 200;

// === Elbow mapping constants (tune these) ===
const int ELBOW_MIN = 10;    // same as servo_joint_L_min_pos
const int ELBOW_MAX = 180;   // same as servo_joint_L_max_pos
const int ELBOW_STEP = 20;

// Mechanical trims (if one horn is off a bit)
const int L_TRIM = -10;  // was your "-10"
const int R_TRIM = 0;

// Canonical elbow angle (replaces servo_joint_L_parking_pos_i as the source of truth)
int elbow = servo_joint_L_parking_pos;

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
const int stepsPerRevolution = 80;
int stepDelay = 1500;
const int stepsPerRevolutionSmall = 60;
int stepDelaySmall = 9500;

const int ButtonPin = 8;
bool OperatingState = true;
bool lastButtonState = HIGH;   // edge detection flag


unsigned int Pos;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(Enable, OUTPUT);
  pinMode(ButtonPin, INPUT_PULLUP);  // internal pull-up
  Serial.begin(9600);
  Bluetooth.begin(9600);

  Serial.println("=== System Powering Up ===");

  //   Initialize PCA9685
  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);

  // Wait for power rails to stabilize
  delay(800);  // gives PCA9685 + servos time to get stable voltage


  //  Now go to default positions smoothly
  wakeUp();

  delay(100);
}


void loop() {
  bool currentButtonState = digitalRead(ButtonPin);

  if (lastButtonState == HIGH && currentButtonState == LOW) {
    OperatingState = !OperatingState;
    Serial.print("Operating State: ");
    Serial.println(OperatingState ? "ON" : "OFF");

    if (!OperatingState) {
      // When turning OFF: move to neutral, but still flush buffer (optional)
      wakeUp();
    } else {
      // When turning ON: flush all old commands from Bluetooth and Serial
      while (Bluetooth.available()) Bluetooth.read();
      while (Serial.available()) Serial.read();
    }

    delay(200); // debounce
  }

  lastButtonState = currentButtonState;

  // Only process commands if ON
  if (OperatingState) {
    if (Bluetooth.available() > 0 || Serial.available() > 0) {
      if (Bluetooth.available() > 0)
        state = Bluetooth.read();
      else
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
        case 'D': demoSequence(); break;
        case 'X': wakeUp(); break;
      }
    }
  }

  delay(20);
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



// Clamp helper
int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Apply mapping to both servos (ALWAYS the same equations)
void applyElbow() {
  // Left servo (channel 0) mirrors the elbow value with trim
  int left  = clampi((ELBOW_MAX - elbow) + L_TRIM, servo_joint_L_min_pos, servo_joint_L_max_pos);

  // Right servo (channel 1) follows the elbow value with trim
  int right = clampi(elbow + R_TRIM,           servo_joint_R_min_pos, servo_joint_R_max_pos);

  HCPCA9685.Servo(0, left);
  HCPCA9685.Servo(1, right);
  Serial.print("Elbow-> L:"); Serial.print(left);
  Serial.print(" R:");        Serial.print(right);
  Serial.print("  e:");       Serial.println(elbow);
}

// Move elbow down (increase canonical value) then apply once
void elbowServoForward() {
  elbow = clampi(elbow + ELBOW_STEP, ELBOW_MIN, ELBOW_MAX);
  applyElbow();
  delay(response_time);
}

// Move elbow up (decrease canonical value) then apply once
void elbowServoBackward() {
  elbow = clampi(elbow - ELBOW_STEP, ELBOW_MIN, ELBOW_MAX);
  applyElbow();
  delay(response_time);
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
  // digitalWrite(Enable, HIGH);
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
  // digitalWrite(Enable, HIGH);
  delay(response_time);
}

void wakeUp() {
  int targets[6] = {
    servo_joint_L_parking_pos,
    servo_joint_R_parking_pos,
    servo_joint_1_parking_pos, // shoulder
    servo_joint_2_parking_pos, // wrist 1
    servo_joint_3_parking_pos, // wrist rotate
    servo_joint_4_parking_pos
  };  // gripper
  

  int current[6] = {
    servo_joint_L_parking_pos_i,
    servo_joint_R_parking_pos_i,
    servo_joint_1_parking_pos_i,
    servo_joint_2_parking_pos_i,
    servo_joint_3_parking_pos_i,
    servo_joint_4_parking_pos_i
  };

  const int steps = 60;       // interpolation steps (higher = smoother)
  const int stepDelayMs = 5; // delay between steps


  int startElbow  = clampi(current[1] - R_TRIM, ELBOW_MIN, ELBOW_MAX);
  int targetElbow = clampi(targets[1] - R_TRIM, ELBOW_MIN, ELBOW_MAX);

  for (int s = 0; s <= steps; s++) {
    int e = map(s, 0, steps, startElbow, targetElbow);

    // Apply mirrored motion
    int left  = clampi((ELBOW_MAX - e) + L_TRIM, servo_joint_L_min_pos, servo_joint_L_max_pos);
    int right = clampi(e + R_TRIM, servo_joint_R_min_pos, servo_joint_R_max_pos);

    HCPCA9685.Servo(0, left);
    HCPCA9685.Servo(1, right);
    delay(stepDelayMs);
  }

  // Update tracking
  elbow = targetElbow;
  servo_joint_L_parking_pos_i = clampi((ELBOW_MAX - elbow) + L_TRIM, servo_joint_L_min_pos, servo_joint_L_max_pos);
  servo_joint_R_parking_pos_i = clampi(elbow + R_TRIM, servo_joint_R_min_pos, servo_joint_R_max_pos);

  // === STEP 3: Move shoulder ===
  for (int s = 0; s <= steps; s++) {
    int pos = map(s, 0, steps, current[2], targets[2]);
    HCPCA9685.Servo(2, pos);
    delay(stepDelayMs);
  }
  servo_joint_1_parking_pos_i = targets[2];

  // === STEP 4: Move wrist 1 ===
  for (int s = 0; s <= steps; s++) {
    int pos = map(s, 0, steps, current[3], targets[3]);
    HCPCA9685.Servo(3, pos);
    delay(stepDelayMs);
  }
  servo_joint_2_parking_pos_i = targets[3];

  // === STEP 5: Move wrist rotate ===
  for (int s = 0; s <= steps; s++) {
    int pos = map(s, 0, steps, current[4], targets[4]);
    HCPCA9685.Servo(4, pos);
    delay(stepDelayMs);
  }
  servo_joint_3_parking_pos_i = targets[4];

  // === STEP 6: Move gripper ===
  for (int s = 0; s <= steps; s++) {
    int pos = map(s, 0, steps, current[5], targets[5]);
    HCPCA9685.Servo(5, pos);
    delay(stepDelayMs);
  }
  servo_joint_4_parking_pos_i = targets[5];
}
void demoSequence() {

  wakeUp();
  delay(500);

  baseRotateLeft();
  delay(300);
  baseRotateRight();
  delay(500);

  elbowServoForward();
  delay(300);
  elbowServoBackward();
  delay(400);

  for (int i = 0; i < 3; i++) shoulderServoForward();
  delay(300);
  for (int i = 0; i < 3; i++) shoulderServoBackward();
  delay(400);

  for (int i = 0; i < 4; i++) wristServoCW();
  delay(300);
  for (int i = 0; i < 4; i++) wristServoCCW();
  delay(400);

  for (int i = 0; i < 2; i++) gripperServoBackward(); 
  delay(500);
  for (int i = 0; i < 2; i++) gripperServoForward(); 
  delay(500);

  wakeUp();
}

