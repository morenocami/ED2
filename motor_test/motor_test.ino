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
boolean on = false, forward = true;

//// motor two
//int enB = 5;
//int in3 = 7;
//int in4 = 6;

char inputButtonState;


void setup()
{
// set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
//  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
//  pinMode(in3, OUTPUT);
//  pinMode(in4, OUTPUT);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  pinMode(12,INPUT);         // Initialize Arduino Digital Pins 12 as input for connecting Pushbutton


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
  inputButtonState = digitalRead(12); //Read the Pushbutton state.

  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  if(on){
    if(euler.y()>10){
      analogWrite(enA, 250);
      delay(100);
      forward=!forward;
    }
    else if(euler.y()<-10){
      analogWrite(enA, 50);
      delay(100);
      forward=!forward;
    }
  }

 
  if (inputButtonState == LOW && on){     
    on = !on;
    analogWrite(enA, 0);
    delay(100);
  } 
  else if(inputButtonState == HIGH && !on){
    on = !on;
    analogWrite(enA, 150);
    delay(100);    
  }
}
