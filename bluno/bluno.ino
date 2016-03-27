void setup() {
  pinMode(13,OUTPUT);
  Serial.begin(115200);
}
int myByte;
void loop() {
  if(Serial.available()>0){
    myByte = Serial.read();
    if(myByte==0){
      Serial.print(0);
    }
    else if(myByte==1){
      digitalWrite(13,HIGH);
      delay(1000);
      digitalWrite(13,LOW);
    }
  }
}
