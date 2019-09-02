// This example code if for half-bridge application. The Motor should be connected to the Output V and GND/VS

// Inhabit pins
int INHU = 6;
int INHV = 5;
int INHW = 3;


//Input pins
int INU = 11;
int INV = 10;
int INW = 9;


void setup() {
  // initialization
  pinMode(INHU, OUTPUT);
  pinMode(INHV, OUTPUT);
  pinMode(INHW, OUTPUT);
  pinMode(INU, OUTPUT);
  pinMode(INV, OUTPUT);
  pinMode(INW, OUTPUT);

  digitalWrite(INHU, LOW);
  digitalWrite(INHV, HIGH);
  digitalWrite(INHW, LOW);
  digitalWrite(INU, LOW);
  digitalWrite(INV, LOW);
  digitalWrite(INV, LOW);

}

void loop() {
//  Output V with PWM
//  Can adjust the frequency and dutycycle by changing delay time.
    digitalWrite(INV, LOW);
    delayMicroseconds(25);
    digitalWrite(INV, HIGH);
    delayMicroseconds(25);
}
