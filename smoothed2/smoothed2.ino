#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
//pins for the motor driver
#define dir1 13
#define pwm1 12
#define dir2 11
#define pwm2 10
#define rB 6
#define lB 7
//pins for the five ultrasonics (22-31)
#define trig1 22
#define echo1 23
#define trig2 24
#define echo2 25
#define trig3 26
#define echo3 27
#define trig4 28
#define echo4 29
#define trig5 30
#define echo5 31

//speaker enable pins
#define speakerObstacle 8 //obstacle alert
#define speakerBackup 9 //curb alert
//ultrasonic sensors @36degrees yields readings of about 40-50
#define dropThreshHold 35
#define obstacleThreshHold 150
//battery indicator LEDs and analog pin
#define gLED 53
#define yLED 52
#define rLED 48
#define batVoltagePin 0

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

Adafruit_BNO055 bno = Adafruit_BNO055();

boolean motorOn = false;
boolean rightB, leftB;
int dutyCycle;
int duration = 0;

//arrays used for sensor data smoothing
const int lowAveraging = 5;
const int midAveraging = 25;
const int highAveraging = 50;
int dropLs[lowAveraging];
int dropRs[lowAveraging];
int lefts[midAveraging];
int rights[midAveraging];
//int faces[lowAveraging];
float batLevels[highAveraging];
int lowIndex;
int midIndex;
int highIndex;
// sum of array elements
int sumDropL;
int sumDropR;
int sumLeft;
int sumRight;
//int sumFace;
float sumBattery;
//running average of array elements
int dropL;
int dropR;
int left;
int right;
//int face;
float batteryLevel;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  // set both motors to move foward
  digitalWrite(dir2, HIGH);
  digitalWrite(dir1, LOW);

  //buttons
  pinMode(lB, INPUT);
  pinMode(rB, INPUT);
  //ultrasonic sensors
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);
  pinMode(trig4, OUTPUT);
  pinMode(echo4, INPUT);
  pinMode(trig5, OUTPUT);
  pinMode(echo5, INPUT);
  //audio feedback modules
  pinMode(speakerObstacle, OUTPUT);
  pinMode(speakerBackup, OUTPUT);

  //BNO055 setup
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.write("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
  Serial.begin(9600);

  //initialize all elements in all smoothing arrays to ZERO
  for (int x = 0; x < lowAveraging; x++) {
//    faces[x] = 0;
    dropLs[x] = 0;
    dropRs[x] = 0;
  }
  for (int x = 0; x < midAveraging; x++) {
    lefts[x] = 0;
    rights[x] = 0;
  }
  for (int x = 0; x < highAveraging; x++) {
    batLevels[x] = 0;
  }
  lowIndex = 0;
  midIndex = 0;
  highIndex = 0;
  sumLeft = 0;
  sumRight = 0;
  sumDropL = 0;
  sumDropR = 0;
//  sumFace = 0;
  sumBattery = 0;

  //Battery status LED
  pinMode(gLED, OUTPUT);
  pinMode(yLED, OUTPUT);
  pinMode(rLED, OUTPUT);
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  rightB = digitalRead(6);
  leftB = digitalRead(7);

  //base speed
  dutyCycle = 75;

  //reading 9DOF
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  //sequentially fires ultrasonic sensors
  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig2, LOW);
  duration = pulseIn(echo2, HIGH, 9000);
  lefts[midIndex] = (duration / 2) / 20;
  
  digitalWrite(trig4, LOW);
  delayMicroseconds(2);
  digitalWrite(trig4, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig4, LOW);
  duration = pulseIn(echo4, HIGH, 3000);
  dropRs[lowIndex] = (duration / 2) / 20;
  
//  digitalWrite(trig5, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trig5, HIGH);
//  delayMicroseconds(8);
//  digitalWrite(trig5, LOW);
//  duration = pulseIn(echo5, HIGH, 5000);
//  faces[lowIndex] = (duration / 2) / 20;
  
  digitalWrite(trig3, LOW);
  delayMicroseconds(2);
  digitalWrite(trig3, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig3, LOW);
  duration = pulseIn(echo3, HIGH, 9000);
  rights[midIndex] = (duration / 2) / 20;
  
  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig1, LOW);
  duration = pulseIn(echo1, HIGH, 3000);
  dropLs[lowIndex] = (duration / 2) / 20;

  batLevels[highIndex] = analogRead(batVoltagePin) * (5.0 / 1023.0) - 0.29;



  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  calcRunAvgs();
  
  
  
  //if there's an obstacle, reduce speed
  if(right==0 || left==0){}
  else{
    if (right < obstacleThreshHold) {
      dutyCycle = dutyCycle - map(right, 30, obstacleThreshHold, 75, 1);
      if(right<50 || right!=0) digitalWrite(speakerObstacle,LOW);
    }
    else if(left < obstacleThreshHold) {
      dutyCycle = dutyCycle - map(left, 30, obstacleThreshHold, 75, 1);
      if(left<50 || left!=0) digitalWrite(speakerObstacle,LOW);
    }
  }
  
  if(left>120 || right>120){
    digitalWrite(speakerObstacle,HIGH);
  }
  
  //User tracking ultrasonic
//  if(face>=50 && face<=100){
//    dutyCycle = dutyCycle - map(face, 50, 100, 1, 75);
//  }
//  else if(face>100){
//    dutyCycle=0;
//  }




  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  if (motorOn) {
    //if there's a drop, brake
    if (dropL > dropThreshHold || dropR > dropThreshHold || dropR==0 || dropL==0) {
      //backup sound
      digitalWrite(speakerBackup, HIGH);
      delay(10);
      digitalWrite(speakerBackup, LOW);
      //stop
      analogWrite(pwm1, 0);
      analogWrite(pwm2, 0);
      delay(1500);
      // set both motors to move backward
      digitalWrite(dir2, LOW);
      digitalWrite(dir1, HIGH);
      //move back
      analogWrite(pwm1, 35);
      analogWrite(pwm2, 35);
      delay(3000);
      //stop
      analogWrite(pwm1, 0);
      analogWrite(pwm2, 0);
      delay(1000);
      // set both motors to move foward
      digitalWrite(dir2, HIGH);
      digitalWrite(dir1, LOW);
      for (int x = 0; x < lowAveraging; x++) {
        dropLs[x] = 20;
        dropRs[x] = 20;
      }
    }
  
    //reduce speed if there is tilt
    dutyCycle = dutyCycle - map(euler.y(), -20, 30, -50, 70);
    
    //zero speed if negative
    if (dutyCycle < 0) {
      dutyCycle = 0;
    }
    
    analogWrite(pwm1, dutyCycle);
    analogWrite(pwm2, dutyCycle);
  }
  //motor BRAKES if either button not pressed
  if ((leftB == LOW || rightB == LOW) && motorOn){
    motorOn = !motorOn;
    analogWrite(pwm1, 0);
    analogWrite(pwm2, 0);
  }
  else if ((leftB == HIGH && rightB == HIGH) && !motorOn){
    motorOn = !motorOn;
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////


  //Battery Status
  if (batteryLevel > 2.40) //about 11.5v
  {
    digitalWrite(gLED, HIGH);
    digitalWrite(yLED, LOW);
    digitalWrite(rLED, LOW);
  }
  else if (batteryLevel > 2.36 && batteryLevel <= 2.40) //about 10.5v to 11.5v
  {
    digitalWrite(gLED, LOW);
    digitalWrite(yLED, HIGH);
    digitalWrite(rLED, LOW); 
  }
  else//about 10.5v
  {
    digitalWrite(gLED, LOW);
    digitalWrite(yLED, LOW);
    digitalWrite(rLED, HIGH);
  }

  //increment array access indexes
  lowIndex++;
  midIndex++;
  highIndex++;
  if(lowIndex==lowAveraging)   lowIndex=0;
  if(midIndex==midAveraging)   midIndex=0;
  if(highIndex==highAveraging) highIndex=0;

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//each for loop adds the elements of an array and sets the corresponding distance variable to an average

void calcRunAvgs() {
//  for (int x = 0; x < lowAveraging; x++) {
//    sumFace += faces[x];
//  }
//  face = sumFace / lowAveraging;
//  sumFace = 0;
  
  for (int x = 0; x < lowAveraging; x++) {
    sumDropL += dropLs[x];
  }
  dropL = sumDropL / lowAveraging;
  sumDropL = 0;

  for (int x = 0; x < lowAveraging; x++) {
    sumDropR += dropRs[x];
  }
  dropR = sumDropR / lowAveraging;
  sumDropR = 0;

  for (int x = 0; x < midAveraging; x++) {
    sumLeft += lefts[x];
  }
  left = sumLeft / midAveraging;
  sumLeft = 0;

  for (int x = 0; x < midAveraging; x++) {
    sumRight += rights[x];
  }
  right = sumRight / midAveraging;
  sumRight = 0;

  for (int x = 0; x < highAveraging; x++) {
    sumBattery += batLevels[x];
  }
  batteryLevel = sumBattery / highAveraging;
  sumBattery = 0;
}
