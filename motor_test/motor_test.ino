#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

// connect motor controller pins to Arduino digital pins
// motor one
int enA = 10;
int in1 = 9;
int in2 = 8;
boolean motorOn= false, forward = true;

//// motor two
int enB = 5;
int in3 = 7;
int in4 = 6;

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

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  pinMode(11,INPUT);         // Initialize Arduino Digital Pins 11 as input for connecting Pushbutton


  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.write("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop()
{
  //check for button press 
  inputButtonState = digitalRead(11); //Read the Pushbutton state.
  //take tilt reading
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//  //fire/read ultrasonic
//  digitalWrite(12, LOW);
//  delayMicroseconds(2);
//  digitalWrite(12, HIGH);
//  delayMicroseconds(8);
//  digitalWrite(12, LOW);
//  duration = pulseIn(13, HIGH, 5000);
//  
//  distance = (duration/2) / 20;
  
  if(motorOn){
    if(euler.y()>10){
      analogWrite(enA, 250);
      analogWrite(enB, 250);
      delay(100);
    }
    else if(euler.y()<-10){
      analogWrite(enA, 50);
      analogWrite(enB, 50);
      delay(100);
    }
    else if(euler.y()>-5 && euler.y()<5){
      analogWrite(enA, 125);
      analogWrite(enB, 125);
      delay(100);
    }
  }

  if (inputButtonState == LOW && motorOn){     
    motorOn= !motorOn;
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    delay(100);
  } 
  else if(inputButtonState == HIGH && !motorOn){
    motorOn= !motorOn;
    analogWrite(enA, 150);
    analogWrite(enB, 150);
    delay(100);    
  }
}
