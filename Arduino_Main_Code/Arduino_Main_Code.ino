#include <Servo.h>
#include <AccelStepper.h>

// Define the pins for your stepper motor
#define X_STEP_PIN 5
#define X_DIR_PIN 2
#define X_ENABLE_PIN 8


int endswitchPin = 6;   // De pin waarop de eindschakelaar is aangesloten

int analogPin = A0; // potentiometer read the Robot arm value
float val = 0;  // variable to store the value read

//Pins actuator
int pinAct2In = 10;     //Actuator1
int pinAct2Out = 11;    //Actuator1
int pinAct1In = 12;     //Actuator2
int pinAct1Out = 13;    //Actuator2

//Servo Camera Gripper
Servo myservo;

// Create an instance of AccelStepper with DRIVER mode (for step/direction control)
AccelStepper stepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);

String fp_num = "focus_9";
uint64_t last_message_time = 0;


void setup() {
  Serial.begin(9600);           //  setup serial
  
  pinMode(pinAct1In, OUTPUT);
  pinMode(pinAct1Out, OUTPUT);
  pinMode(pinAct2In, OUTPUT);
  pinMode(pinAct2Out, OUTPUT);

  pinMode(analogPin, INPUT);
  pinMode(endswitchPin, INPUT_PULLUP);


  myservo.attach(9); // Servo attach to pin 9
}

// void EndSwitch(){
//   switchState = digitalRead(endswitchPin);

//   // Toon de status van de eindschakelaar in de seriële monitor
//   if (switchState == LOW) {
//     Serial.println("Eindschakelaar is ingedrukt (NO gesloten).");
//   } else {
//     Serial.println("Eindschakelaar is niet ingedrukt (NO open).");
//   }
//   delay(500);
// }

void ServoCameraClose(int Position){ //0 degree: open  //90 degree: close
  for (int pos=30; pos <=89; pos++){
    myservo.write(pos);
    delay(15);
  }
}

void ServoCameraOpen(int Position){ //0 degree: open  //90 degree: close
  myservo.write(30);
}

void ExtendAct(int actuator)
{
  Serial.print("extend");
  if (actuator==1){ // actuator 1 is opening actuator
    digitalWrite(pinAct1In, HIGH);
    digitalWrite(pinAct1Out, LOW);
    //delay(8000);      
    }
  else {
    digitalWrite(pinAct2In, HIGH);
    digitalWrite(pinAct2Out, LOW);
    //delay(4000);      
    }
}

void RetractAct(int actuator)
{
  Serial.print("retract");
  if (actuator==1){ // actuator 1 is opening actuator
    digitalWrite(pinAct1In, LOW);
    digitalWrite(pinAct1Out, HIGH);
    //delay(8000);   
    }
  else {
    digitalWrite(pinAct2In, LOW);
    digitalWrite(pinAct2Out, HIGH);
    //delay(4000);    
    }
}

void StilstandAct(int actuator) {
  if (actuator==1){
    digitalWrite(pinAct1In, LOW);
    digitalWrite(pinAct1Out, LOW);
  }
  else {
    digitalWrite(pinAct2In, LOW);
    digitalWrite(pinAct2Out, LOW);
  }
}

void loop() {
  /*
  val = (analogRead(analogPin) );  // read the input pin
  val = val * (5.0 / 1023.0);     
  */

  if (val == 0.0){
    val = Serial.parseFloat();
    Serial.print("Received value: ");
    Serial.println(val);      // Print the value
  //Serial.println(val);          // debug value
  }  

  if (val >= 0.0 && val <= 0.4) { //Nothing
  } else if (val >= 0.5 && val <= 1.0) { //Open Camera Gripper
    ServoCameraOpen(30);
  } else if (val >= 1.1 && val <= 1.5) { //Close Camera Gripper
    ServoCameraClose(89);
  } else if (val >= 1.6 && val <= 2.0) { //Extend Actuator 1
    if (digitalRead(endswitchPin) == LOW) {
      ExtendAct(1);
    }
    else {
      StilstandAct(1);
    }
  } else if (val >= 2.1 && val <= 2.5) { //Retract Actuator 1
    if (digitalRead(endswitchPin) == LOW) {
      RetractAct(1);
    }
    else {
      StilstandAct(1);
    }
  } else if (val >= 2.6 && val <= 3.0) { //Extend Actuator 2
    ExtendAct(2);
  } else if (val >= 3.1 && val <= 3.5) { //Retract Actuator 2
    RetractAct(2);
  } else if (val >= 3.6 && val <= 4.0) { //Activate glue
  
   if (Serial.available() > 0) {
        // Read the number of steps from the serial input
        long steps = Serial.parseInt();
        last_message_time = millis();
        // Check if steps value is valid (not 0)
        if (steps != 0) {
            // Move the stepper by the given number of steps
            stepper.move(steps);
            stepper.runToPosition();  // Block until the move is completed

            // Send a "Done" message to the PC after the movement is completed
            delay(1);
            Serial.println("Done");
        if (steps == 0){Serial.println("Done");}    
        }
    }
    if (millis() - last_message_time > 10 * 1000) {
  // Serial.println("off");
      digitalWrite(X_ENABLE_PIN, HIGH);
        }




  } else if (val >= 4.1 && val <= 4.5) {

  } else if (val >= 4.6 && val <= 5.0) {

  }
  val = 0.0;
}
