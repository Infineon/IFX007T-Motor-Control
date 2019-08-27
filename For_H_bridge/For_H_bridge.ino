//This code is for h bridge configuration. The motor should be connected to output V and output W.
// Inhabit pins
int INHU = 6;
int INHV = 5;
int INHW = 3;
byte incomingbyte;
//Input pins
int INU = 11;
int INV = 10;
int INW = 9;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(INHU, OUTPUT);
  pinMode(INHV, OUTPUT);
  pinMode(INHW, OUTPUT);
  pinMode(INU, OUTPUT);
  pinMode(INV, OUTPUT);
  pinMode(INW, OUTPUT);
  digitalWrite(INHV, HIGH);
  digitalWrite(INHW, HIGH);
  digitalWrite(INHU, LOW);
  digitalWrite(INW, LOW);// Output W is LOW.
}

void loop() {
//get PWM signal with 10kHz at output V. 
//Frequency and Duty cycle can be changed by changing the delay time.
   digitalWrite(INV, LOW);
   delayMicroseconds(50);
   digitalWrite(INV, HIGH);
   delayMicroseconds(50);

}
