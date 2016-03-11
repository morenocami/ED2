 #include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();
 // motor one
int dir1 = 13;
int pwm1 = 12;

//motor two
int dir2 = 11;
int pwm2 = 10;

boolean motorOn= false, forward = true;

//Buttons
char rightB, leftB;

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
  pinMode(7,INPUT);
  pinMode(6,INPUT);
  
    if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.write("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  rightB = digitalRead(7);
  leftB = digitalRead(6);
  dutyCycle=75;
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  if(motorOn){
     dutyCycle = dutyCycle - map(euler.y(),-2,-30,0,60);

//    if(distance<5){
//      dutyCycle -=50;
//    }
//    else if(distance<10){
//      dutyCycle -=25;
//    }
    
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

