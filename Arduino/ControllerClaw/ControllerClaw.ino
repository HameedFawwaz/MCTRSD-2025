/* Pin initialization */
int topButton = 2;
int rightButton = 3;
int leftButton = 4;
int downButton = 5;
int rightLimit = 6;
int leftLimit = 7;

//define motor control pins
#define in1 8 //5
#define in2 9 //4
#define in3 10 //3
#define in4 11 //2

#define ClawPin 12 
#define ServoPin 13

#define VRX_PIN  A1 // Arduino pin connected to VRX pin
#define VRY_PIN  A0 // Arduino pin connected to VRY pin

// Define Tresholds for controller

#define LEFT_THRESHOLD  400
#define RIGHT_THRESHOLD 800
#define UP_THRESHOLD    400
#define DOWN_THRESHOLD  800

// Define Commenad identifiers

#define COMMAND_NO     0x00
#define COMMAND_LEFT   0x01
#define COMMAND_RIGHT  0x02
#define COMMAND_UP     0x04
#define COMMAND_DOWN   0x08

//motor pins
const int motorA_forward = in1;
const int motorA_backward = in2;
const int motorB_forward = in3;
const int motorB_backward = in4;
const int outputThreshold = 20;

const int ClawPin = ClawPin;
const int ServoPin = ServiPin;

// Initial values for the claw and riser system

int xValue = 0 ; // To store value of the X axis
int yValue = 0 ; // To store value of the Y axis
int command = COMMAND_NO;

int claw_pos = 0;
int rise_pos = 0;

const int speed = 200;

#include <Servo.h>

// Initialize claw and riser

Servo claw;
Servo riser;

void setup() {
  Serial.begin(9600);
  pinMode(topButton, INPUT_PULLUP);  
  pinMode(rightButton, INPUT_PULLUP);  
  pinMode(leftButton, INPUT_PULLUP);  
  pinMode(downButton, INPUT_PULLUP);  

  // motor control pins
  pinMode(motorA_forward, OUTPUT);
  pinMode(motorA_backward, OUTPUT);
  pinMode(motorB_forward, OUTPUT);
  pinMode(motorB_backward, OUTPUT);


  claw.attach(ClawPin);
  riser.attach(RiserPin);

}

void loop() {
  if (digitalRead(topButton) == LOW) {
    Serial.println("Up D-pad");
    driveForward(speed);
    //move forward
  }

  if (digitalRead(rightButton) == LOW) {
    Serial.println("Right D-pad");
    turnRight(speed);
    //turn right
  }

  if (digitalRead(leftButton) == LOW) {
    Serial.println("Left D-pad");
    turnLeft(speed);
    //move back
  }

  if (digitalRead(downButton) == LOW) {
    Serial.println("Down D-pad");
    driveBackward(speed);
    //move backward
 
  }

  if (digitalRead(rightLimit) == LOW) {
    Serial.println("Right Limit");
    //do something

  }

  if (digitalRead(leftLimit) == LOW) {
    Serial.println("Left Limit");
    //do something else

  }

  // read analog X and Y analog values
  xValue = analogRead(VRX_PIN);
  yValue = analogRead(VRY_PIN);

  // converts the analog value to commands
  // reset commands
  command = COMMAND_NO;

  // check left/right commands
  if (xValue < LEFT_THRESHOLD)
    command = command | COMMAND_LEFT;

  else if (xValue > RIGHT_THRESHOLD)
    command = command | COMMAND_RIGHT;

  // check up/down commands
  if (yValue < UP_THRESHOLD)
    command = command | COMMAND_UP;
  else if (yValue > DOWN_THRESHOLD)
    command = command | COMMAND_DOWN;

  // NOTE: AT A TIME, THERE MAY BE NO COMMAND, ONE COMMAND OR TWO COMMANDS

  // print command to serial and process command
  if (command & COMMAND_LEFT) {
    Serial.println("COMMAND LEFT");

    claw_pos++;
    claw.write(claw_pos);
    // TODO: add your task here
  }

  if (command & COMMAND_RIGHT) {
    Serial.println("COMMAND RIGHT");


    
    claw_pos--;
    claw.write(claw_pos);
    // TODO: add your task here
  }

  if (command & COMMAND_UP) {
    Serial.println("COMMAND UP");

    rise_pos++;
    riser.write(rise_pos);
    // TODO: add your task here
  }

  if (command & COMMAND_DOWN) {
    Serial.println("COMMAND DOWN");

    rise_pos--;
    riser.write(rise_pos);
    // TODO: add your task here
  }

}

/* Drive Functions, motor by is backwards for some reason */
void driveForward(int speed) {
  analogWrite(motorA_forward, speed);
  analogWrite(motorB_forward, 0);
  analogWrite(motorA_backward, 0);
  analogWrite(motorB_backward, speed);
}

void driveBackward(int speed) {
  analogWrite(motorA_forward, 0);
  analogWrite(motorB_forward, speed);
  analogWrite(motorA_backward, speed);
  analogWrite(motorB_backward, 0);
}

//stop when adjusted
void stopMotors() {
  analogWrite(motorA_forward, 0);
  analogWrite(motorB_forward, 0);
  analogWrite(motorA_backward, 0);
  analogWrite(motorB_backward, 0);
}

void turnLeft(int speed) { //assuming motor A is left wheels
  analogWrite(motorA_forward, speed);
  analogWrite(motorB_forward, speed);
  analogWrite(motorA_backward, 0);
  analogWrite(motorB_backward, 0);
}

void turnRight(int speed) { //assuming motor A is left wheels
  analogWrite(motorA_forward, 0);
  analogWrite(motorB_forward, 0);
  analogWrite(motorA_backward, speed);
  analogWrite(motorB_backward, speed);
}