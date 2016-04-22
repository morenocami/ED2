void setup() {
  pinMode(2,OUTPUT);
  Serial.begin(9600);
}


int count=0;
int value = HIGH;


void loop() {
  digitalWrite(2,HIGH);
  delay(10);
  
  digitalWrite(2,LOW);
  delay(5000);
}
