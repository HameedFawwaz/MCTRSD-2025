#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

//define motor control pins
#define in1 5 //5
#define in2 4 //4
#define in3 3 //3
#define in4 2 //2

//create i2c object
MPU6050 mpu;

//PID parameters
double Kp = 1.5; //tested proportional values: 
double Ki = 0.05; //tested integral values: 
double Kd = 0.5; //tested derivative values: 

//base angle at init
double setPoint = 0.0;

//PID vars
double error = 0.0;
double previousError = 0.0;
double integral = 0.0;
double derivative = 0.0;
double outputPID = 0.0;
unsigned long lastTime = 0;

//motor pins
const int motorA_forward = in1;
const int motorA_backward = in2;
const int motorB_forward = in3;
const int motorB_backward = in4;
const int outputThreshold = 20;

//kalman filter for gyro 
class KalmanFilter {
public:
  KalmanFilter() {
    Q_angle = 0.001;    // acc variance
    Q_bias  = 0.003;    // gyro bias
    R_measure = 0.03;   // measurement variance
    angle = 0.0;        // base angle
    bias = 0.0;         // reset bias
    P[0][0] = 0.0;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 0.0;
  }

  double getAngle(double newAngle, double newRate, double dt) {
    //prediction
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance and project
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // innovation
    double y = newAngle - angle;
    // covariance
    double S = P[0][0] + R_measure;
    // kalman gain
    double K0 = P[0][0] / S;
    double K1 = P[1][0] / S;

    // update step
    angle += K0 * y;
    bias  += K1 * y;

    // update error covariance
    double P00_temp = P[0][0];
    double P01_temp = P[0][1];

    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;

    return angle;
  }

private:
  double Q_angle, Q_bias, R_measure;
  double angle, bias, rate;
  double P[2][2];
};

// call kalman class
KalmanFilter kalman;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // init mpu6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
  
  // motor control pins
  pinMode(motorA_forward, OUTPUT);
  pinMode(motorA_backward, OUTPUT);
  pinMode(motorB_forward, OUTPUT);
  pinMode(motorB_backward, OUTPUT);
  
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0; // unit conversion to s
  if (dt <= 0) return; //dont time travel

  // accelerometer
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  // just comment acceleration if sensor isn't centered or do geometry to adjust offset
  double accAngle = atan2((double)ax, sqrt((double)ay * ay + (double)az * az)) * 180.0 / PI;
  
  //gyro
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  // degrees per second and change axis
  double gyroRate = gx / 131.0;

  // update kalman data
  double filteredAngle = kalman.getAngle(accAngle, gyroRate, dt);

  //error calc
  error = setPoint - filteredAngle;
  integral += error * dt;
  derivative = (error - previousError) / dt;
  outputPID = Kp * error + Ki * integral + Kd * derivative;

  //output testing
  Serial.print("Acc Angle: ");
  Serial.print(accAngle);
  Serial.print(" | Filtered Angle: ");
  Serial.print(filteredAngle);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | PID Output: ");
  Serial.println(outputPID);
  
  previousError = error;
  lastTime = now;
  
  //pid control out to motor
  int motorSpeed = constrain(abs(outputPID), outputThreshold, 255);
  if (outputPID > outputThreshold) {
    driveBackward(motorSpeed);
  } else if (outputPID < -outputThreshold) {
    driveForward(motorSpeed);
  } else {
    stopMotors();
  }
  
  delay(10);
}

// drive functions
void driveForward(int speed) {
  analogWrite(motorA_forward, speed);
  analogWrite(motorB_forward, speed);
  analogWrite(motorA_backward, 0);
  analogWrite(motorB_backward, 0);
}

void driveBackward(int speed) {
  analogWrite(motorA_forward, 0);
  analogWrite(motorB_forward, 0);
  analogWrite(motorA_backward, speed);
  analogWrite(motorB_backward, speed);
}

//stop when adjusted
void stopMotors() {
  analogWrite(motorA_forward, 0);
  analogWrite(motorB_forward, 0);
  analogWrite(motorA_backward, 0);
  analogWrite(motorB_backward, 0);
}
