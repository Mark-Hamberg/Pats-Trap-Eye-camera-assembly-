#include <Servo.h>

int analogPin = A5; // potentiometer read the Robot arm value
int val = 0;  // variable to store the value read

//Pins actuator
int pinAct1In = 12;
int pinAct1Out = 13;
int pinAct2In = xx;
int pinAct2Out = xx;

//Servo Camera Gripper
Servo myservo;
int PosServo = 0;


void setup() {
  Serial.begin(9600);           //  setup serial
  
  pinMode(pinAct1In, OUTPUT);
  pinMode(pinAct1Out, OUTPUT)
  pinMode(pinAct2In, OUTPUT);
  pinMode(pinAct2Out, OUTPUT)

  myservo.attach(9); // Servo attach to pin 9
}

void ServoCamera(Position){
  myservo.write(Position)
}


void ExtendAct(actuator)
{
  if (actuator==1){ // actuator 1 is opening actuator
    digitalWrite(pinAct1In, LOW);
    digitalWrite(pinAct1Out, HIGH);
    delay(8000)
  }
  else {
    digitalWrite(pinAct2In, LOW);
    digitalWrite(pinAct2Out, HIGH);
    delay(4000)
  }
}

void RetractAct(actuator)
{
  if (actuator==1){ // actuator 1 is opening actuator
    digitalWrite(pinAct2In, HIGH);
    digitalWrite(pinAct2Out, LOW);
    delay(8000)
  }
  else {
    digitalWrite(pinAct2In, HIGH);
    digitalWrite(pinAct2Out, LOW);
    delay(4000)
  }
}


void loop() {
  val = analogRead(analogPin);  // read the input pin
  Serial.println(val);          // debug value

  switch (reading) {
    case 0.5 ... 1.0 : { //Open Camera Gripper
      
      }
      break;

    case 1.1 ... 1.5 : { //Close Camera Gripper
      
      }
      break;

    case 1.6 ... 2.0: { //Extend Actuator 1
        ExtendAct1(1);
      }
      break;

    case 2.1 ... 2.5: { //Retract Actuator 1
      RetractAct(1)
      }
      break;

    case 2.6 ... 3.0: { //Extend Actuator 2
      ExtendAct1(2);
      }
      break;

    case 3.1 ... 3.5: { //Retract Actuator 2
      RetractAct(2)
      }
      break;

    case 3.6 ... 4.0: { //Activte glue
      }
      break;
      
      

    default: {
      //No message
      }

  }



}