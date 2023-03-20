#include <SharpIR.h>
#define Kp 1 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 100// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define MaxSpeed 255// max speed of the robot
#define BaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define SensorCount  8     // number of sensors used


#define ir A0
#define model 1080
// ir: the pin where your sensor is attached
// model: an int that determines your sensor:  1080 for GP2Y0A21Y
//                                            20150 for GP2Y0A02Y
//                                            (working distance range according to the datasheets)
SharpIR SharpIR(ir, model);



#define speedturn 180

//#define rightMotor1 5
//#define rightMotor2 6
//#define rightMotorPWM 7
//#define leftMotor1 4
//#define leftMotor2 3
//#define leftMotorPWM 2
//#define motorPower A3
int rightMotorSpeed;
int leftMotorSpeed;
int ena = 5;
int in1 = 6;
int in2 = 7;
int in3 = 9;
int in4 = 8;
int enb = 10;

const uint8_t SensorPins[SensorCount] = {A0, A1, A2, A3, A4, A5, 2, 3};
//unsigned int sensorValues[SensorCount];

int lastError = 0;
unsigned int Sensor[8];




void setup()
{
  Serial.begin(115200);
  //  pinMode(rightMotor1, OUTPUT);
  //  pinMode(rightMotor2, OUTPUT);
  //  pinMode(rightMotorPWM, OUTPUT);
  //  pinMode(leftMotor1, OUTPUT);
  //  pinMode(leftMotor2, OUTPUT);
  //  pinMode(leftMotorPWM, OUTPUT);

  for (int i = 0; i < SensorCount ; i++) {
    pinMode(SensorPins[i], INPUT);
  }

  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enb, OUTPUT);
  //pinMode(motorPower, OUTPUT);

  //wait();
  delay(2000); // wait for 2s to position the bot before entering the main loop
}



void loop()
{
  //  for(int i =0; i<8; i++){
  //    Serial.print(i);
  //    Serial.print(" :");
  //    Serial.println(digitalRead(SensorPins[i]));
  //  }

  //  Serial.println("");
  //  delay(1000);

  //  return;


  int Position = 0;
  int sensor = 0;
  for (int i = 0 ; i < SensorCount; i++) {
    Position += i * 1000 * digitalRead(SensorPins[i]);
    sensor += digitalRead(SensorPins[i]);
  }
  Position /= sensor;
  // Serial.println(Position);
  // delay(500);
  // return;


  int dis=SharpIR.distance();
  if ( dis <= 13) {
    Serial.println(dis);
    while (Position > 6500) {
      move(0, speedturn, 1);
      move(1, speedturn, 0);
    }
    return;
  }

  if (Position < 400) {
    //    move(0, speedturn, 1);
    //    move(1, speedturn, 1);
    move(0, rightMotorSpeed, 1);
    move(1, leftMotorSpeed, 1);
    return;
  }

  //int error = Position ;
  //int  error = Position - (SensorCount-1)*1000/2;
  int  error = Position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  //  Serial.println(error);
  //  return;

  rightMotorSpeed = BaseSpeed + motorSpeed;
  leftMotorSpeed = BaseSpeed - motorSpeed;

  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;

  move(0, rightMotorSpeed, 1);
  move(1, leftMotorSpeed, 1);
}


void move(int motor, int Speed, int Direction) {
  //digitalWrite(motorPower, HIGH); //disable standby

  boolean inPin1 = HIGH;
  boolean inPin2 = LOW;

  if (Direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  if (Direction == 0) {
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if (motor == 0) {
    digitalWrite(in1, inPin1);
    digitalWrite(in2, inPin2);
    analogWrite(ena, Speed);
  }
  if (motor == 1) {
    digitalWrite(in3, inPin1);
    digitalWrite(in4, inPin2);
    analogWrite(enb, Speed);
  }
}
