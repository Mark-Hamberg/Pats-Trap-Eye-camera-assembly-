#include <Servo.h>

int analogPin = A0; // potentiometer read the Robot arm value
float val = 0;  // variable to store the value read

//Pins actuator
int pinAct1In = 12;
int pinAct1Out = 13;
int pinAct2In = 10;
int pinAct2Out = 11;

//Servo Camera Gripper
Servo myservo;

void setup() {
  Serial.begin(9600);           //  setup serial
  
  pinMode(pinAct1In, OUTPUT);
  pinMode(pinAct1Out, OUTPUT);
  pinMode(pinAct2In, OUTPUT);
  pinMode(pinAct2Out, OUTPUT);

  myservo.attach(9); // Servo attach to pin 9
}

void ServoCamera(int Position){ //0 degree: open  //90 degree: close
  myservo.write(Position);   
  }

void ExtendAct(int actuator)
{
  if (actuator==1){ // actuator 1 is opening actuator
    digitalWrite(pinAct1In, LOW);
    digitalWrite(pinAct1Out, HIGH);
    delay(8000);    }
  else {
    digitalWrite(pinAct2In, LOW);
    digitalWrite(pinAct2Out, HIGH);
    delay(4000);    }
}

void RetractAct(int actuator)
{
  if (actuator==1){ // actuator 1 is opening actuator
    digitalWrite(pinAct2In, HIGH);
    digitalWrite(pinAct2Out, LOW);
    delay(8000);      }
  else {
    digitalWrite(pinAct2In, HIGH);
    digitalWrite(pinAct2Out, LOW);
    delay(4000);      }
}

void loop() {
  val = (analogRead(analogPin) / 1023);  // read the input pin
  Serial.println(val);          // debug value

  if (val >= 0.0 && val <= 0.4) { //Nothing
  }
  if (val >= 0.5 && val <= 1.0) { //Open Camera Gripper
    ServoCamera(0);
  }
  if (val >= 1.1 && val <= 1.5) { //Close Camera Gripper
    ServoCamera(90);
  }
  if (val >= 1.6 && val <= 2.0) { //Extend Actuator 1
    ExtendAct(1);
  }
  if (val >= 2.1 && val <= 2.5) { //Retract Actuator 1
    RetractAct(1);
  }
  if (val >= 2.6 && val <= 3.0) { //Extend Actuator 2
    ExtendAct(2);
  }
  if (val >= 3.1 && val <= 3.5) { //Retract Actuator 2
    RetractAct(2);
  }
  if (val >= 3.6 && val <= 4.0) { //Activate glue
  }
  if (val >= 4.1 && val <= 4.5) {
  }
  if (val >= 4.6 && val <= 5.0) {
  }
}