// connect motor controller pins to Arduino digital pins
// motor one
int enA = 10;
int in1 = 9;
int in2 = 8;

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
}

void loop()
{  
  inputButtonState = digitalRead(12); //Read the Pushbutton state.
 
  if (inputButtonState == HIGH) 
  {     
    analogWrite(enA, 0);
    delay(100);
  } 
  else 
  {
    analogWrite(enA, 200);
    delay(100);    
  }
}
