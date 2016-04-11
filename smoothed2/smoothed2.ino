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
#define dropThreshHold 35
#define obstacleThreshHold 50

Adafruit_BNO055 bno;

boolean motorOn = false;

boolean rightB, leftB;
int dutyCycle;
int duration = 0;

//arrays used for sensor data smoothing
const int numReadings = 10;
int dropLs[numReadings];
int dropRs[numReadings];
int lefts[numReadings];
int rights[numReadings];
int index;
// sum of array elements
int sumDropL;
int sumDropR;
int sumLeft;
int sumRight;
//running average of array elements
int dropL;
int dropR;
int left;
int right;
//Battery status
int analogPin = 0;
int reading = 0;
float voltageT = 0;
#define gLED 32
#define yLED 33
#define rLED 34



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
  else {
    bno = Adafruit_BNO055();
    delay(1000);
    bno.setExtCrystalUse(true);
  }

  Serial.begin(9600);


  //initialize all elements in all smoothing arrays to ZERO
  for (int x = 0; x < numReadings; x++) {
    lefts[x] = 0;
    rights[x] = 0;
    dropLs[x] = 0;
    dropRs[x] = 0;
  }
  index = 0;
  sumLeft = 0;
  sumRight = 0;
  sumDropL = 0;
  sumDropR = 0;

  //Battery status LED
  pinMode(gLED, OUTPUT);
  pinMode(yLED, OUTPUT);
  pinMode(rLED, OUTPUT);
}






////////////////
//main loop
////////////////

void loop() {
  //Battery vars
  reading = 0;
  float voltage = 0;
  float voltageBat = 0;

  rightB = digitalRead(6);
  leftB = digitalRead(7);

  //base speed
  dutyCycle = 75;

  //reading 9DOF
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  //sequentially fires ultrasonic sensors
  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig1, LOW);
  duration = pulseIn(echo1, HIGH, 4000);
  dropLs[index] = (duration / 2) / 20;

  digitalWrite(trig4, LOW);
  delayMicroseconds(2);
  digitalWrite(trig4, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig4, LOW);
  duration = pulseIn(echo4, HIGH, 4000);
  dropRs[index] = (duration / 2) / 20;

  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig2, LOW);
  duration = pulseIn(echo2, HIGH, 4000);
  lefts[index] = (duration / 2) / 20;

  digitalWrite(trig3, LOW);
  delayMicroseconds(2);
  digitalWrite(trig3, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig3, LOW);
  duration = pulseIn(echo3, HIGH, 4000);
  rights[index] = (duration / 2) / 20;

  ////////////////////////////
  //calculate running averages
  ////////////////////////////
  calcRunAvgs();

  Serial.println(dropL);
  Serial.println(dropR);
  Serial.println("\n");
  //if there's a drop, brake; else if there's an obstacle, reduce speed
  if ((dropL > dropThreshHold) || (dropR > dropThreshHold)) {
    dutyCycle = 0;
  }
  if (right < obstacleThreshHold && right != 0) {
    dutyCycle = dutyCycle - map(right, 15, 50, 75, 1);

  }
  else if (left < obstacleThreshHold && left != 0) {
    dutyCycle = dutyCycle - map(left, 15, 50, 75, 1);

  }


  //when buttons pressed, motor on
  if (motorOn) {
    //reduce speed if there is tilt
    dutyCycle = dutyCycle - map(euler.y(), -2, -30, 0, 60);

    //zero speed if negative
    if (dutyCycle < 0) {
      dutyCycle = 0;
    }

    analogWrite(pwm1, dutyCycle);
    analogWrite(pwm2, dutyCycle);

    delay(10);
  }

  if ((leftB == LOW || rightB == LOW) && motorOn)
  {
    motorOn = !motorOn;
    analogWrite(pwm1, 0);
    analogWrite(pwm2, 0);
  }

  else if ((leftB == HIGH && rightB == HIGH) && !motorOn)
  {
    motorOn = !motorOn;
  }

  index++;
  if (index == 10)index = 0;


  //Battery Status
  for (int i = 0; i < 10; i++)
  {

    reading = analogRead(analogPin);
    voltage = reading * (5.0 / 1023.0);
    voltage = voltage - 0.29; // Differnce error
    voltageT += voltage;
  }
  voltageBat = 0;
  voltageBat = voltageT / 10;
  voltageT = 0;
  Serial.println(voltageBat);
  delay(200);
  //Battery LED Status
  Serial.println(voltageBat);
  if (voltageBat > 2.75) //11.5v+
  {
    digitalWrite(gLED, HIGH);
    digitalWrite(yLED, LOW);
    digitalWrite(rLED, LOW);
    Serial.println("green");
  }
  else if (voltageBat <= 2.75 && voltageBat > 2.69) //11.5v-10.5v
  {
    digitalWrite(gLED, LOW);
    digitalWrite(yLED, HIGH);
    digitalWrite(rLED, LOW);
    Serial.println("yellow");
  }
  else//10.5v-
  {
    digitalWrite(gLED, LOW);
    digitalWrite(yLED, LOW);
    digitalWrite(rLED, HIGH);
    Serial.println("red");
  }

}


//each for loop adds the elements of an array and sets the corresponding distance variable to an average
void calcRunAvgs() {
  for (int x = 0; x < numReadings; x++) {
    sumDropL += dropLs[x];
  }
  dropL = sumDropL / numReadings;
  sumDropL = 0;

  for (int x = 0; x < numReadings; x++) {
    sumDropR += dropRs[x];
  }
  dropR = sumDropR / numReadings;
  sumDropR = 0;

  for (int x = 0; x < numReadings; x++) {
    sumLeft += lefts[x];
  }
  left = sumLeft / numReadings;
  sumLeft = 0;

  for (int x = 0; x < numReadings; x++) {
    sumRight += rights[x];
  }
  right = sumRight / numReadings;
  sumRight = 0;
}

//Mapping function for floats
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}





