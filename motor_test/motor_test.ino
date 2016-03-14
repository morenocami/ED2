#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define dir1 13
#define pwm1 12
#define dir2 11
#define pwm2 10
#define rB 6
#define lB 7
#define trig1 2
#define echo1 3
#define trig2 4
#define echo2 5
#define trig3 8
#define echo3 9
#define trig4 A0
#define echo4 A1

Adafruit_BNO055 bno;

boolean motorOn = false, forward = true;

boolean rightB, leftB;
int dutyCycle;

void setup() {
  // put your setup code here, to run once:
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);

  // set both motors to move foward
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, LOW);

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

  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig1, LOW);
  int duration = pulseIn(echo1, HIGH, 5000);
  int left = (duration / 2) / 20;
  Serial.print("Distance Left: ");
  Serial.println(left);


  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig3, LOW);
  duration = pulseIn(echo2, HIGH, 5000);
  int right = (duration / 2) / 20;
  Serial.print("Distance Right: ");
  Serial.println(right);

  digitalWrite(trig3, LOW);
  delayMicroseconds(2);
  digitalWrite(trig3, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig3, LOW);
  duration = pulseIn(echo3, HIGH, 5000);
  int drop = (duration / 2) / 20;
  Serial.print("Distance Drop: ");
  Serial.println(drop);
  Serial.println("");

//  digitalWrite(trig4, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trig4, HIGH);
//  delayMicroseconds(8);
//  digitalWrite(trig4, LOW);
//  duration = pulseIn(echo4, HIGH, 5000);
//  int right = (duration / 2) / 20;
//  Serial.print("Distance Right: ");
//  Serial.println(distance);
//  Serial.println("");

  delay(300);
  
  int dropThreshHold = 100;
  if(drop>dropThreshHold){
    dutyCycle=0;
  }
  
  int obstacleTreshHold;
  if(right<obstacleThreshHold){
    dutyCycle = dutyCycle - map(right, 20, 100, 75, 1);    
  }
  else if(left<obstacleThreshHold){
    dutyCycle = dutyCycle - map(left, 20, 100, 75, 1);    
  }












    // put your main code here, to run repeatedly:
    rightB = digitalRead(6);
    leftB = digitalRead(7);
  
    dutyCycle=75;
  
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
    if(motorOn){
       dutyCycle = dutyCycle - map(euler.y(),-2,-30,0,60);
  
      if(distance<5){
        dutyCycle -=50;
      }
      else if(distance<10){
       dutyCycle -=25;
      }
      
      if(dutyCycle<0){dutyCycle=0;}
      
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

