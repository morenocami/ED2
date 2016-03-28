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
//pins for the four ultrasonics (22-29)
#define trig1 22
#define echo1 23
#define trig2 24
#define echo2 25
#define trig3 26
#define echo3 27
#define trig4 28
#define echo4 29
//speaker enable pins
#define speakerOne 30
#define speakerTwo 31
//ultrasonic sensors @36degrees yields readings of about 40-50
#define dropThreshHold 40
#define obstacleThreshHold 50

Adafruit_BNO055 bno;

boolean motorOn = false, forward = true;

boolean rightB, leftB;
int dutyCycle;

int duration=0;
int sumLeft=0;
int sumRight=0;
int sumDropL=0;
int sumDropR=0;
int count=0, count2=0;

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


  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);
  pinMode(trig4, OUTPUT);
  pinMode(echo4, INPUT);
  
  pinMode(speakerOne, OUTPUT);

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.write("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  else{
    bno = Adafruit_BNO055();
    delay(1000);
    bno.setExtCrystalUse(true);
  }

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  rightB = digitalRead(6);
  leftB = digitalRead(7);
  //base speed
  dutyCycle=75;
  //reading 9DOF
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  sumLeft=0;
  sumRight=0;
  sumDropL=0;
  sumDropR=0;
  
  //fires ultrasonic sensors five times
  for(count = 0; count<5; count++){
    digitalWrite(trig1, LOW);
    delayMicroseconds(2);
    digitalWrite(trig1, HIGH);
    delayMicroseconds(8);
    digitalWrite(trig1, LOW);
    duration = pulseIn(echo1, HIGH, 4000);
    int dropL = (duration / 2) / 20;
    sumDropL += dropL;
    
    digitalWrite(trig2, LOW);
    delayMicroseconds(2);
    digitalWrite(trig2, HIGH);
    delayMicroseconds(8);
    digitalWrite(trig2, LOW);
    duration = pulseIn(echo2, HIGH, 4000);
    int left = (duration / 2) / 20;
    sumLeft += left;
    
    digitalWrite(trig4, LOW);
    delayMicroseconds(2);
    digitalWrite(trig4, HIGH);
    delayMicroseconds(8);
    digitalWrite(trig4, LOW);
    duration = pulseIn(echo4, HIGH, 4000);
    int dropR = (duration / 2) / 20;
    sumDropR += dropR;
    
    digitalWrite(trig3, LOW);
    delayMicroseconds(2);
    digitalWrite(trig3, HIGH);
    delayMicroseconds(8);
    digitalWrite(trig3, LOW);
    duration = pulseIn(echo3, HIGH, 4000);
    int right = (duration / 2) / 20;
    sumRight += right;
  }

  //calculate average of distances
  int dropL = sumDropL/count;
  int dropR = sumDropR/count;
  int left = sumLeft/count;
  int right = sumRight/count;
          
  if(count2==10){
    Serial.println("Left: ");
    Serial.println(left);
    Serial.println("Right: ");
    Serial.println(right);
//    Serial.println("dropLeft: ");
//    Serial.println(dropL);
//    Serial.println("dropRight: ");
//    Serial.println(dropR);
    Serial.println("");
    Serial.println("");
    count2=0;
//    if(dropL>dropThreshHold || dropR>dropThreshHold){
//      digitalWrite(speakerOne,HIGH);
//      digitalWrite(speakerOne,LOW);
//      Serial.println(dropL);
//      Serial.println(dropR);
//    }
  }
  else{
    count2++;
  }
  
  //if there's a drop, brake; else if there's an obsacle, reduce speed
  if((dropL>dropThreshHold && dropL!=0) || (dropR>dropThreshHold && dropR!=0)){
    dutyCycle=0;
  }
  else if(right<obstacleThreshHold && right!=0){
    dutyCycle = dutyCycle - map(right, 15, 50, 75, 1);    
  }
  else if(left<obstacleThreshHold && left!=0){
    dutyCycle = dutyCycle - map(left, 15, 50, 75, 1);    
  }


  //when buttons pressed, motor on
  if(motorOn){
    //reduce speed if there is tilt
    dutyCycle = dutyCycle - map(euler.y(),-2,-30,0,60);
    
    //reduce speed if obstacles ahead
    
    //zero speed if negative
    if(dutyCycle<0){
      dutyCycle=0;
    }
    
    analogWrite(pwm1, dutyCycle);
    analogWrite(pwm2, dutyCycle);
    delay(100);
    Serial.println(dutyCycle);
  }

  if ((leftB == LOW || rightB == LOW) && motorOn)
  {
    motorOn= !motorOn;
    analogWrite(pwm1, 0);
    analogWrite(pwm2, 0);
  }

  else if((leftB == HIGH && rightB == HIGH) && !motorOn)
  {
    motorOn= !motorOn;
  }
}

