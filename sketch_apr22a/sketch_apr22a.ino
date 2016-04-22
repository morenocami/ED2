#define trig5 22
#define echo5 23

int duration = 0;
int face=0;
void setup() {
  // put your setup code here, to run once:
  pinMode(trig5, OUTPUT);
  pinMode(echo5, INPUT);
   Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(trig5, LOW);
  delayMicroseconds(2);
  digitalWrite(trig5, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig5, LOW);
  duration = pulseIn(echo5, HIGH, 8000);
  face = (duration / 2) / 20;
  Serial.println(face);
  delay(10);
}
