 
void setup() {
  Serial.begin(9600);
  pinMode(A2, INPUT);
}

void loop(){
  int tcrt = analogRead(A2);
  Serial.print("\nhigh ");
  Serial.println(tcrt);
  delay(100);
}

