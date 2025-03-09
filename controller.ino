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

//motor pins
const int motorA_forward = in1;
const int motorA_backward = in2;
const int motorB_forward = in3;
const int motorB_backward = in4;
const int outputThreshold = 20;

const int speed = 200;


void setup() {\
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

  //stopMotors();

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