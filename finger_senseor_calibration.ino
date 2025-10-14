
#include<Wire.h>

//Create thumb Sensors
int pinkie = 0; //Pinkie thumb
int finger = 0; //finger thumb
int thumb = 0; //Index thumb

int pinkie_Data = A1;
int finger_Data = A2;
int thumb_Data = A3;



/*Autotune flex parameter
  For Debug Mode. Check the upper and lowe limit of the flex sensors
  3 Flex sensors used. Thumb, Middle, Pinkie
*/
int thumb_high = 960;
int thumb_low = 870;
int finger_high = 920;
int finger_low = 827;
int pinkie_high = 950;
int pinkie_low = 850;

//Stop Caliberating the Flex Sensor when complete
bool bool_caliberate = true;

//How often to send values to the Robotic Arm
int response_time = 100;

void setup() {
  pinMode(3, OUTPUT);

  Serial.begin(9600);
  delay(1000);

}
void loop() {

  /*
    Note: Serial.print() would send all values the robotic arm using via bluetooth.
  */
  pinMode(3, HIGH); //Use basic LED as visual indicator if value being sent

  debug_flex(); //Debug Mode on/off
  Serial.println("test");


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
    delay(response_time);

  }
  if (pinkie <= pinkie_low ) {
    Serial.print("p");
    delay(response_time);
  }


  // thumb 1 - thumb (Base Rotation)
  if (thumb >= thumb_high) {
    Serial.print("T");
    delay(response_time);
  }

  if (thumb <= thumb_low) {
    Serial.print("t");
    delay(response_time);
  }

  // finger 1 - Claw Bend/Open
  if (finger >= finger_high) {
    Serial.print("F");
    delay(response_time);
  }

  if (finger <= finger_low) {
    Serial.print("f");
    delay(response_time);
  }
  else {
    delay(5);
  }
}


void debug_flex() {
  //Sends value as a serial monitor to port
  //thumb (Claw open / close)
  Serial.print("Thumb: ");
  Serial.print(thumb);
  Serial.print("\t");
  //  //thumb Params
  Serial.print("thumb High: ");
  Serial.print(thumb_high);
  Serial.print("\t");
  Serial.print("T Low: ");
  Serial.print(thumb_low);
  Serial.print("\t");

  //finger (Claw Further)
  Serial.print("finger: ");
  Serial.print(finger);
  Serial.print("\t");

  //  finger Params
  Serial.print("finger High: ");
  Serial.print(finger_high);
  Serial.print("\t");
  Serial.print("finger Low: ");
  Serial.print(finger_low);
  Serial.print("\t");

  //Pinkie (Claw Further)
  Serial.print("Pinkie: ");
  Serial.print(pinkie);
  Serial.print("\t");

  //  //Pinkie Params
  Serial.print("Pinkie High: ");
  Serial.print(pinkie_high);
  Serial.print("\t");
  Serial.print("Pinkie Low: ");
  Serial.print(pinkie_low);
  Serial.print("\t");
  Serial.pr