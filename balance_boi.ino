//Motor setup code is built on Sparkfun's example code:
//https://github.com/sparkfun/SIK-Guide-Code/blob/master/SIK_Circuit_5A-MotorBasics/SIK_Circuit_5A-MotorBasics.ino

#include "MPU9250.h"

//motor stuff
const int AIN1 = 13, AIN2 = 12, PWMA = 11; 
const int PWMB = 10, BIN2 = 9, BIN1 = 8;

//PID stuff
double setpointY = -0.42;
double bias = 0.0;
double iError;
double previousMeasure = 0.0;
double alpha;
double filtTime = 0.02;
double filtMeasure = setpointY;
double alphaD;
double filtTimeD = 0.0;
double filtD = 0.0;
double dt = 0.004;

double kp = 0.75;
double ki = 0.0;
double kd = 0.014;

//MPU9250 stuff
MPU9250 IMU(Wire,0x68);
int status;

/********************************************************************************/
void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  
  status = IMU.begin();
  if (status < 0) 
    while(1)
      Serial.println("no MPU");
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
}

/********************************************************************************/
void loop() {
  IMU.readSensor();
  alpha = dt / (filtTime + dt);
  filtMeasure = ((1 - alpha) * filtMeasure) + (alpha * IMU.getAccelY_mss());  
//  Serial.print("filtered y accel: ");
//  Serial.println(filtMeasure);

  if ((filtMeasure > -5) && (filtMeasure < 5))
    bothMotor(255 * pid(filtMeasure, setpointY));
  else{
    bothMotor(0.0);
    pid(filtMeasure, setpointY);
  }
  delay(dt * 1000);
}

/********************************************************************************/
double pid(double current, double setpoint) {
  double error = setpoint - current;
  iError = iError + (error * dt);
  
  double pControl = kp * error;
  double iControl = ki * iError;
  double dControl = kd * (-(current - previousMeasure) / dt);
  previousMeasure = current;
  
  alphaD = dt / (filtTimeD + dt);
  filtD = ((1 - alphaD) * filtD) + (alphaD * dControl);
  
  double command = pControl + iControl + filtD;
  
  Serial.print("  command: ");
  Serial.println(command);
//  Serial.print("  p: ");
//  Serial.print(pControl);
//  Serial.print("  i: ");
//  Serial.print(iControl);
//  Serial.print("  d: ");
//  Serial.println(dControl);
  
  if (command > 1.0)
    return 1.0;
  else if (command < -1.0)
    return -1.0;
  else
    return command;
}

/********************************************************************************/
void bothMotor(int motorSpeed) {
  if (motorSpeed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else if (motorSpeed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMA, min(abs(motorSpeed) + bias, 255));
  analogWrite(PWMB, min(abs(motorSpeed) + bias, 255));
}

