#include <Arduino.h>
#include <Servo.h>
#include <AccelStepper.h>


// Defines pins numbers
// -- Note: D3 not used as that port is blown on my nano
// A, B and C are the stepper motors for Base, SHoulder and Elbow
#define stepEnable 2
#define servoPin 4
#define stepA 5
#define dirA 6
#define stepB 7
#define dirB 8
#define stepC 9
#define dirC 10

// objects
Servo servo;
AccelStepper stepperA(AccelStepper::DRIVER, stepA, dirA);

// state
int pulseLength = 1500; // delay: 500 too short, 1000 ok
int direction = 0;
long numSteps = 200;
int servoAngle = 0;     //servo angle which can vary from 0 - 180
int abc = 1;
int motorsOff = 0;

// limit switches
const byte interruptPin = 14;

void setup() {  
  // steppers
  pinMode(stepEnable, OUTPUT);
  digitalWrite(stepEnable, LOW);  

  //pinMode(stepA, OUTPUT);
  pinMode(stepB, OUTPUT);
  pinMode(stepC, OUTPUT);
  //pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(dirC, OUTPUT);

  stepperA.setAcceleration(400.0);
  stepperA.setMaxSpeed(1000); // steps per sec 
  //stepperA.moveTo(300);

  // limit switches
  pinMode(interruptPin, INPUT_PULLUP);

  servo.attach(servoPin, 1000, 2000);

  Serial.begin(9600);
  Serial.println("Press button to advance stepper.");
  Serial.println("  D: direction, 1-9: Num Rotations");
  Serial.println("  S: Slower, F: Faster");
}

void servoTest() {
/*  for(servoAngle = 0; servoAngle <= 180; servoAngle++)  //increment the angle from 0 to 180, in steps of 1
  { 
    servo.write(servoAngle);                        //set the servo angle and it will move to it
    delay(20);                                          //wait 20ms before moving to the next position
  }
  for (servoAngle = 180; servoAngle >= 0; servoAngle--) //decrement the angle from 180 to 0, in steps of 1 
  {
    servo.write(servoAngle);                        //set the servo angle and it will move to it
    delay(20);                                          //wait 20ms before moving to the next position
  }
*/
  servo.write(90);                                  //move servo to 60 deg
  delay(500);                                           //wait for 500ms   
  
  servo.write(180);                                 //move servo to 120 deg
  delay(500);                                           //wait for 500ms  
  
  servo.write(90);                                 //move servo to 180 deg
  delay(500);                                           //wait for 500ms  
}

void handleCommand() {

  while (!Serial.available()) { ; }
  byte c = Serial.read();

  if ( c == 'a' ) {
    abc = abc ^ 1;
  }
  if ( c == 'b' ) {
    abc = abc ^ 2;
  }
  if ( c == 'c' ) {
    abc = abc ^ 4;
  }
  Serial.print("ABC: "); Serial.println(abc);

  if ( c == 'd' ) {
    direction = direction ^ 1;
    Serial.print("Direction: ");
    Serial.println(direction);
    //digitalWrite(dirA, direction);
    digitalWrite(dirB, direction);
    digitalWrite(dirC, direction);
  }

  if ( c >= '1' && c <= '9') {
    long steps = (long(c) - '0') * 200;
    numSteps = steps;
  }

  if ( c == 's') {
    pulseLength += 100;
    Serial.print("Pulse length delay: ");
    Serial.println(pulseLength);
  }

  if ( c == 'f') {
    pulseLength -= 100;
    Serial.print("Pulse length delay: ");
    Serial.println(pulseLength);
  }

  if ( c == 'g') 
    servoTest(); // grab 

  if ( c == 'x') {
    motorsOff = motorsOff ^ 1;
    digitalWrite(stepEnable, motorsOff); 
    if (motorsOff == 1)
      Serial.println("DISABLED STEPPERS!!!");
    if (motorsOff == 0)
      Serial.println("Steppers ON");
  }
}

void loop() {
  
  //stepperA.run();
  // stepperA.runToPosition();

  // delay(300);
  stepperA.moveTo(400);
  stepperA.runToPosition();
  delay(600);
  stepperA.setAcceleration(1000);
  stepperA.setMaxSpeed(2400);
  stepperA.moveTo(-400);
  stepperA.runToPosition();

  handleCommand();

  for(int x = 0; x < numSteps; x++) {
    Serial.print(digitalRead(interruptPin));

    if (digitalRead(interruptPin) == LOW) {
      // stepperA.stop();
      Serial.println("HOMING STOP!");
      // break; // stop moror spinning - homing switch triggered
    }    

    // step pulse
    //if (abc & 1)
    //   digitalWrite(stepA, HIGH); 
    if (abc & 2)
      digitalWrite(stepB, HIGH); 
    if (abc & 4)
      digitalWrite(stepC, HIGH);

    delayMicroseconds(pulseLength); // 500 is too fast    

    // if (abc & 1)
    //   digitalWrite(stepA, LOW); 
    if (abc & 2)
      digitalWrite(stepB, LOW); 
    if (abc & 4)
      digitalWrite(stepC, LOW);

    delayMicroseconds(pulseLength);
  }
  
  Serial.print(".");
}
