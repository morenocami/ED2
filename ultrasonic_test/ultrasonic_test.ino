#define dir1 13
#define pwm1 12
#define dir2 11
#define pwm2 10
#define rB 6
#define lB 7
#define trig1 22
#define echo1 23
#define trig2 24
#define echo2 25
#define trig3 26
#define echo3 27
#define trig4 28
#define echo4 29
int sumLeft=0;
int sumRight=0;
int sumDropL=0;
int sumDropR=0;
int count=0;


void setup() {
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);
  pinMode(trig4, OUTPUT);
  pinMode(echo4, INPUT);

  Serial.begin(9600);
}

void loop() {
  
  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig1, LOW);
  int duration = pulseIn(echo1, HIGH, 4000);
  int left = (duration / 2) / 20;
  sumLeft += left;
  

  digitalWrite(trig4, LOW);
  delayMicroseconds(2);
  digitalWrite(trig4, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig4, LOW);
  duration = pulseIn(echo4, HIGH, 4000);
  int right = (duration / 2) / 20;
  sumRight += right;
  

  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig2, LOW);
  duration = pulseIn(echo2, HIGH, 4000);
  int dropL = (duration / 2) / 20;
  sumDropL += dropL;

  
  digitalWrite(trig3, LOW);
  delayMicroseconds(2);
  digitalWrite(trig3, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig3, LOW);
  duration = pulseIn(echo3, HIGH, 4000);
  int dropR = (duration / 2) / 20;
  sumDropR += dropR;
  
  count++;
  if(count==5){
    sumLeft=sumLeft/5;
    sumRight=sumRight/5;
    sumDropL = sumDropL/5;
    sumDropR = sumDropR/5;
    Serial.println("Left: ");
    Serial.println(sumLeft);
    Serial.println("DropLeft: ");
    Serial.println(sumDropL);
    Serial.println("DropRight: ");
    Serial.println(sumDropR);
    Serial.println("Right: ");
    Serial.println(sumRight);
    Serial.println("");
    count=0;
    sumLeft=0;
    sumRight=0;
    sumDropL=0;
    sumDropR=0;
  }
  delay(1000);
}
