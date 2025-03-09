#include <AFMotor.h>
#include <IRremote.h>

//IR Receiver 
const int RECV_PIN = 9;
IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long key_value = 0;

//Motor Driver
int IN1 = 2
int IN2 = 3
int IN3 = 4
int IN4 = 5

//Ultrasonic sensor
const int trigPin = 12;
const int echoPin = 11;
float duration, distance;



void setup() {

  //Motor Driver
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //Ultrasonic Sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //IR Sensor
  Serial.begin(9600);
  irrecv.enableIRIn();
  irrecv.blink13(true);
}

void loop(){


  /* IR Receiver*/
  if (irrecv.decode(&results)){
 
    if (results.value == 0XFFFFFFFF)
      results.value = key_value;

    /* Currently just prints button pressed, will need to change to what happens
    when button is pressed */
    switch(results.value){
      case 0xFFA25D:
      Serial.println("CH-");
      break;
      case 0xFF629D:
      Serial.println("CH");
      break;
      case 0xFFE21D:
      Serial.println("CH+");
      break;
      case 0xFF22DD:
      Serial.println("|<<");
      break;
      case 0xFF02FD:
      Serial.println(">>|");
      break ;  
      case 0xFFC23D:
      Serial.println(">|");
      break ;               
      case 0xFFE01F:
      Serial.println("-");
      break ;  
      case 0xFFA857:
      Serial.println("+");
      break ;  
      case 0xFF906F:
      Serial.println("EQ");
      break ;  
      case 0xFF6897:
      Serial.println("0");
      break ;  
      case 0xFF9867:
      Serial.println("100+");
      break ;
      case 0xFFB04F:
      Serial.println("200+");
      break ;
      case 0xFF30CF:
      Serial.println("1");
      break ;
      case 0xFF18E7:
      Serial.println("2");
      break ;
      case 0xFF7A85:
      Serial.println("3");
      break ;
      case 0xFF10EF:
      Serial.println("4");
      break ;
      case 0xFF38C7:
      Serial.println("5");
      break ;
      case 0xFF5AA5:
      Serial.println("6");
      break ;
      case 0xFF42BD:
      Serial.println("7");
      break ;
      case 0xFF4AB5:
      Serial.println("8");
      break ;
      case 0xFF52AD:
      Serial.println("9");
      break ;      
    }
        key_value = results.value;
        irrecv.resume(); 

    
  /* Motor Driver */

  // Rotate the Motor A clockwise
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(2000);
  // Motor A
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  delay(500);
  // Rotate the Motor B clockwise
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(2000);
  // Motor B
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  delay(500);
  // Rotates the Motor A counter-clockwise
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(2000);
  // Motor A
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  delay(500);
  // Rotates the Motor B counter-clockwise
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(2000);
  // Motor B
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  delay(500);


  /* Ultrasonic Sensor */
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);

  }
}





/*Random guy online */
/*
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

char command;

void setup() 
{       
 Serial.begin(9600);  //Set the baudrate for Bluetooth module.
}

void loop(){
 if(Serial.available() > 0){ 
   command = Serial.read(); 
   Stop(); //initialize with motors stoped
   
   //Serial.println(command);
   switch(command){
   case 'F':  
     forward();
     break;
   case 'B':  
      back();
     break;
   case 'L':  
     left();
     break;
   case 'R':
     right();
     break;
   }
 } 
}

void forward()
{
 motor1.setSpeed(255); //Define maximum velocity
 motor1.run(FORWARD); //rotate the motor clockwise
 motor2.setSpeed(255); //Define maximum velocity
 motor2.run(FORWARD); //rotate the motor clockwise
 
 motor3.setSpeed(255); //Define maximum velocity
 motor3.run(FORWARD); //rotate the motor clockwise
 motor4.setSpeed(255); //Define maximum velocity
 motor4.run(FORWARD); //rotate the motor clockwise
}

void back()
{
 motor1.setSpeed(255); 
 motor1.run(BACKWARD); //rotate the motor counterclockwise
 motor2.setSpeed(255); 
 motor2.run(BACKWARD); //rotate the motor counterclockwise

 motor3.setSpeed(255); 
 motor3.run(BACKWARD); //rotate the motor counterclockwise
 motor4.setSpeed(255); 
 motor4.run(BACKWARD); //rotate the motor counterclockwise
}

void left()
{
 motor1.setSpeed(255); //Define maximum velocity
 motor1.run(FORWARD); //rotate the motor clockwise
 motor2.setSpeed(255); //Define maximum velocity
 motor2.run(BACKWARD); //rotate the motor counterclockwise

 motor3.setSpeed(255); //Define maximum velocity
 motor3.run(FORWARD); //rotate the motor clockwise
 motor4.setSpeed(255); //Define maximum velocity
 motor4.run(BACKWARD); //rotate the motor counterclockwise
}

void right()
{
 motor1.setSpeed(255); //Define maximum velocity
 motor1.run(BACKWARD); //rotate the motor counterclockwise
 motor2.setSpeed(255); //Define maximum velocity
 motor2.run(FORWARD); //rotate the motor clockwise

 motor3.setSpeed(255); //Define maximum velocity
 motor3.run(BACKWARD); //turn motor1 off
 motor4.setSpeed(255); //Define maximum velocity
 motor4.run(FORWARD); //rotate the motor clockwise
}

void Stop()
{
 motor1.setSpeed(0);
 motor2.run(RELEASE); //turn motor1 off
 motor2.setSpeed(0);
 motor2.run(RELEASE); //turn motor2 off

 motor3.setSpeed(0);
 motor3.run(RELEASE); //turn motor3 off
 motor4.setSpeed(0);
 motor4.run(RELEASE); //turn motor4 off
}

*/