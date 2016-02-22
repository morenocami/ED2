#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define enA 10
#define in1 9
#define in2 8
#define enB 5
#define in3 7
#define in4 6
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define trig 12
#define echo 13
#define touchLeft 4
#define touchRight 3

Adafruit_BNO055 bno = Adafruit_BNO055();
boolean motorOn= false; 
boolean forward = true;
int dutyCycle=0;
char inputButtonState;


void setup()
{
// set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  pinMode(touchLeft,INPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);


  if(!bno.begin())
  {
    //Serial.write("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

///////////////////////////////////////////////////////////////////////////
void loop()
{
  //check for button press 
  inputButtonState = digitalRead(touchLeft);

  if (inputButtonState == LOW && motorOn){     
    motorOn= !motorOn;
    dutyCycle = 0;
  } 
  else if(inputButtonState == HIGH && !motorOn){
    motorOn= !motorOn;
    dutyCycle = 150;
  }
    
  dutyCycle -= adjustForTilt();
  dutyCycle -= adjustForDistance();
  
  analogWrite(enA, dutyCycle);
  analogWrite(enB, dutyCycle);
  delay(100);
}

///////////////////////////////////////////////////////////////////////////
int adjustForDistance(){
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig, LOW);
  int duration = pulseIn(echo, HIGH, 5000);
  int distance = constrain(distance, 1, 20);
  distance = (duration/2) / 20;
  
  distance = map(distance,1,20,125,25);
  return distance;
}

///////////////////////////////////////////////////////////////////////////
int adjustForTilt(){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  
  
  if(euler.y()>10){
    return 0;
  }
  else if(euler.y()<-10){
    return 50;
  }
  else if(euler.y()>-5 && euler.y()<5){
    analogWrite(enA, 125);
    analogWrite(enB, 125);
    delay(100);
  }
}

