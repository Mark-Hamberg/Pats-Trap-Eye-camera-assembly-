#include <Servo.h>
#include <AccelStepper.h>

// Define the pins for your stepper motor
#define X_STEP_PIN 7
#define X_DIR_PIN 4
#define X_ENABLE_PIN 8

// Define the GPIO pins for the IR sensors
#define SENSOR1_PIN 2
#define SENSOR2_PIN 3

/*######################################################### Initiate ################################################################*/
int endswitchPin = 6;   // End switch pin actuator

int analogPin = A1; // potentiometer read the Robot arm value
float val = 0;  // variable to store the value read

//Pins actuator
int pinAct1In = 12;     //Actuator1
int pinAct1Out = 13;    //Actuator1
int pinAct2In = 10;     //Actuator2
int pinAct2Out = 11;    //Actuator2

int Raiseerror = 53; //fout als er geen pcb is

//Servo Camera Gripper
Servo myservo;
int currentAngle = 0;   //current angle of the servomotor

// Create an instance of AccelStepper with DRIVER mode (for step/direction control)
AccelStepper stepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
String fp_num = "focus_9";
uint64_t last_message_time = 0;

/*######################################################### Set-Up ################################################################*/
void setup() {
  Serial.begin(115200);           //  setup serial
  pinMode(52, OUTPUT);
  pinMode(SENSOR1_PIN, INPUT); // Set SENSOR1_PIN as input
  pinMode(SENSOR2_PIN, INPUT); // Set SENSOR2_PIN as input

  //Set-up all actuators outputs
  pinMode(pinAct1In, OUTPUT);
  pinMode(pinAct1Out, OUTPUT);
  pinMode(pinAct2In, OUTPUT);
  pinMode(pinAct2Out, OUTPUT);

  pinMode(analogPin, INPUT);
  pinMode(endswitchPin, INPUT_PULLUP);

  myservo.attach(9); // Servo attach to pin 9
  currentAngle = myservo.read();

  pinMode(X_ENABLE_PIN, OUTPUT);
  digitalWrite(X_ENABLE_PIN, LOW);  // Enable the stepper driver

  Serial.setTimeout(100);
  stepper.setMaxSpeed(500);      // Set the maximum speed (steps per second)
  stepper.setAcceleration(5000);   // Set the acceleration (steps per second^2)
}
bool isSensorTriggered(int sensorPin) {
  return digitalRead(sensorPin) == HIGH; // Returns true if sensor gives HIGH signal
}
/*######################################################### Functions ################################################################*/

//void EndSwitch(){
 //  switchState = digitalRead(endswitchPin);

   // Toon de status van de eindschakelaar in de seriële monitor
//   if (switchState == LOW) {
///     Serial.println("Eindschakelaar is ingedrukt (NO gesloten).");
  // } else {
 //   Serial.println("Eindschakelaar is niet ingedrukt (NO open).");
  // }
 //  delay(500);
//}
void checkSensors() {
  bool sensor1State = !isSensorTriggered(SENSOR1_PIN);
  bool sensor2State = !isSensorTriggered(SENSOR2_PIN);

  // Output the state of each sensor
  if (sensor1State && sensor2State) {
    digitalWrite(Raiseerror, LOW);
  } else if (sensor1State) {
    Serial.println("5 pcb left");
    digitalWrite(Raiseerror, LOW);
    
  } else if (sensor2State) {
    Serial.println("Sensor 2 detected something!");
    digitalWrite(Raiseerror, HIGH);
  } else {
    Serial.println("No detection by either sensor.");
    digitalWrite(Raiseerror, HIGH);

  }
}

void ServoCamera(int newAngle){   // 0 degree: open  // 90 degree: close
  int stepDelay = 50;             // The delay between steps(ms), if increased: slow down gripper
  currentAngle = myservo.read();  // Update the current position

  if (newAngle > currentAngle) {  // If it is closing: move at a delayed speed
    for (int pos = currentAngle; pos <= newAngle; pos++) {
      myservo.write(pos);
      delay(stepDelay);
      Serial.println("Open.");
    }
  } else {                        // If it is opening: move at a maximum speed
    for (int pos = currentAngle; pos >= newAngle; pos--) {
      myservo.write(pos);
    }
  }
}

void ExtendAct(int actuator)
{
  Serial.print("extend");
  if (actuator==1){             // open actuator 1
    digitalWrite(pinAct1In, HIGH);
    digitalWrite(pinAct1Out, LOW);
    //delay(8000);      
    }  else {                   // open actuator 2
    digitalWrite(pinAct2In, HIGH);
    digitalWrite(pinAct2Out, LOW);
    //delay(4000);      
    }
}

void RetractAct(int actuator)
{
  Serial.print("retract");
  if (actuator==1){           // close actuator 1
    digitalWrite(pinAct1In, LOW);
    digitalWrite(pinAct1Out, HIGH);
    //delay(8000);   
    }  else {                 // open actuator 2
    digitalWrite(pinAct2In, LOW);
    digitalWrite(pinAct2Out, HIGH);
    //delay(4000);    
    }
}

void StilstandAct(int actuator) {
  if (actuator==1){           // stop actuator 1
    digitalWrite(pinAct1In, LOW);
    digitalWrite(pinAct1Out, LOW);
  }  else {                   // stop actuator 2
    digitalWrite(pinAct2In, LOW);
    digitalWrite(pinAct2Out, LOW);
  }
}

void GlueStation(long steps){       //full rotation of stepper motor is 1600 steps
  digitalWrite(X_ENABLE_PIN, LOW);
  stepper.move(steps);              // Move the assigned amount of steps
  stepper.runToPosition();          // Wait until the position is reached
  Serial.println("Forward Done");

  digitalWrite(X_ENABLE_PIN, HIGH);
  delay(1000);
  digitalWrite(X_ENABLE_PIN, LOW);

  stepper.move(-steps);              // Move the assigned amount of steps
  stepper.runToPosition();          // Wait until the position is reached
  Serial.println("Backwards Done");
  digitalWrite(X_ENABLE_PIN, HIGH);
}

/*######################################################### Loop ################################################################*/

void loop() {
  
  val = (analogRead(analogPin) );  // read the input pin
  val = val * (5.0 / 1023.0);     
  
  checkSensors(); // Call the function to check sensor states
  /*
  if (val == 0.0){
    val = Serial.parseFloat();
    Serial.print("Received value: ");
    Serial.println(val);      // Print the value
  //Serial.println(val);          // debug value
  }
  */  

  if (val >= 0.0 && val <= 0.4) { //Nothing
  } else if (val >= 0.5 && val <= 1.0) { //Open Camera Gripper
    ServoCamera(20);
  } else if (val >= 1.1 && val <= 1.5) { //Close Camera Gripper
    ServoCamera(91);
  } else if (val >= 1.6 && val <= 2.0) { //Slightly Open Camera Gripper
    ServoCamera(85);
  } else if (val >= 2.1 && val <= 2.5) { //Extend Actuator 1
    Serial.println("extend 1 ");
    if (digitalRead(endswitchPin) == HIGH) {ExtendAct(1);}       else {StilstandAct(1);}
  } else if (val >= 2.6 && val <= 3.0) { //Retract Actuator 1
    if (digitalRead(endswitchPin) == HIGH) {RetractAct(1);}      else {StilstandAct(1);}
  } else if (val >= 3.1 && val <= 3.5) { //Extend Actuator 2
    ExtendAct(2);
  } else if (val >= 3.6 && val <= 4.0) { //Retract Actuator 2
    RetractAct(2);
  } else if (val >= 4.1 && val <= 4.5) { //Activate glue
    /*digitalWrite(52, HIGH);
    delay(1000); // Wacht 1000 milliseconden (1 seconde)*/
    GlueStation(1350);
    digitalWrite(52, LOW);
  } else if (val >= 4.6 && val <= 5.0) {  //Stop both actuators
    StilstandAct(1);
    StilstandAct(2);
  }
  
  val=0.0;
  // Turn the stpper motor (glue station) off when it is not being used
  last_message_time = millis();                   // Read the number of steps from the serial input
  if (millis() - last_message_time > 10 * 1000) {
    Serial.println("off");
    digitalWrite(X_ENABLE_PIN, HIGH);
  }
}
