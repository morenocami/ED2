#define trig 22
#define echo 23



//arrays used for sensor data smoothing
const int lowAveraging = 5;
int dropLs[lowAveraging];
int lowIndex;
int sumDropL;

//running average of array elements
int dropL;

void setup() {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  for (int x = 0; x < lowAveraging; x++) {
    dropLs[x] = 0;
  }
  lowIndex = 0;
  sumDropL = 0;
  
  Serial.begin(9600);
}

int duration = 0;



void loop() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH, 3000);
  dropLs[lowIndex] = (duration / 2) / 20;
  
  calcRunAvgs();
  
  delay(10);
  Serial.print("Drop left: ");
  Serial.println(dropL);
  Serial.print("\n\n\n\n\n");
  delay(10);

  //increment array access indexes
  lowIndex++;
  if(lowIndex==lowAveraging)   lowIndex=0;
}



void calcRunAvgs() {
  for (int x = 0; x < lowAveraging; x++) {
    sumDropL += dropLs[x];
  }
  dropL = sumDropL / lowAveraging;
  sumDropL = 0;
}
