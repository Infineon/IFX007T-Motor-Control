////This example code is for sensorless BLDC motor control 
//Please be noticed it has to be modified for diffenrent motor 

//Define Inputs & Outputs
#define PWM_U 11
#define PWM_V 10
#define PWM_W 9
#define INH_U 6
#define INH_V 5
#define INH_W 3

// Based on ADC phase voltage inputs:
#define BEMF_U A3
#define BEMF_V A2
#define BEMF_W A1
#define ADC_VS A0


//Define Motor parameters
#define MotorPoles 8
#define HallPoleShift 0



//PrintInfo


//StartUp - Commutation-Counts to switch over to closed-loop
#define OpenLoopToClosedLoopCount 50



uint8_t Commutation = 0;
uint8_t BEMF_phase = A1;    //Hex value???
uint8_t ClosedLoop = 0;
uint8_t DutyCycle = 150;
uint8_t Dir = 0;
uint32_t V_neutral = 0;
void setup() {

  Serial.begin(115200);
  
  setPwmFrequency(PWM_U, 1);
  setPwmFrequency(PWM_V, 1);
  setPwmFrequency(PWM_W, 1);

  // Set ADC sampling time faster in order to be fast enough to detect commutation pulse:
  setADCspeedFast();
  
  // put your setup code here, to run once:

  pinMode(INH_U, OUTPUT);
  pinMode(INH_V, OUTPUT);
  pinMode(INH_W, OUTPUT);
  pinMode(PWM_U, OUTPUT);
  pinMode(PWM_V, OUTPUT);
  pinMode(PWM_W, OUTPUT);

  V_neutral = (((uint32_t)analogRead(ADC_VS) * DutyCycle) >> 8);
}

void loop() {

  uint16_t i = 5000;
  uint8_t CommStartup = 0;
  
  // Startup procedure: start rotating the field slowly and increase the speed
  while (i>1000) {
    delayMicroseconds(i);
    Commutation = CommStartup;
    UpdateHardware (CommStartup,0);
    CommStartup++;
    if (CommStartup==6) CommStartup=0;
    i=i-20;
    
  }

  // main loop:
  while(1) {
    DoCommutation();
  }
  
}



void DoCommutation() {
V_neutral = (((uint32_t)analogRead(ADC_VS) * DutyCycle) >> 8);
  //V_neutral = analogRead(BEMF_phase);

  if (Dir == 0) {
    switch (Commutation) {
      case 0:
        for (int i=0; i< 20; i++) {
          if (analogRead(BEMF_W)>V_neutral) i-=1;
        }
        if (analogRead(BEMF_W) < V_neutral) {
          Commutation = 1;
          UpdateHardware(Commutation,0);          
        }
        break;
      case 1:
      
        for (int i=0; i< 20; i++) {
          if (analogRead(BEMF_V)<V_neutral) i-=1;
        }
        if (analogRead(BEMF_V) > V_neutral) { 
          Commutation=2;
          UpdateHardware(Commutation,0);
        }
        break;
      case 2:
      
        for (int i=0; i< 20; i++) {
          if (analogRead(BEMF_U)>V_neutral) i-=1;
        }
        if (analogRead(BEMF_U) < V_neutral) { 
          Commutation=3;
          UpdateHardware(Commutation,0);
        }
        break;
      case 3:
      
        for (int i=0; i< 20; i++) {
          if (analogRead(BEMF_W)<V_neutral) i-=1;
        }
        if (analogRead(BEMF_W) > V_neutral) { 
          Commutation=4;
          UpdateHardware(Commutation,0);
        }
        break;
      case 4:
      
        for (int i=0; i< 20; i++) {
          if (analogRead(BEMF_V)>V_neutral) i-=1;
        }
        if (analogRead(BEMF_V) < V_neutral) { 
          Commutation=5;
          UpdateHardware(Commutation,0);
        }
        break;
      case 5:
      
        for (int i=0; i< 20; i++) {
          if (analogRead(BEMF_U)<V_neutral) i-=1;
        }
        if (analogRead(BEMF_U) > V_neutral) { 
          Commutation=0;
          UpdateHardware(Commutation,0);
        }
        break;
      defailt:
      break;
      
    }
  }
}


//defining commutation steps according to HALL table
void UpdateHardware(uint8_t CommutationStep, uint8_t Dir) {

  // update neutral voltage:
  V_neutral = (int)(((uint32_t)analogRead(ADC_VS) * DutyCycle) >> 8);
 
  
  //CW direction
  if (Dir == 0) {

    switch (CommutationStep) {
      case 0:
        digitalWrite(INH_U, HIGH);
        digitalWrite(INH_V, HIGH);
        digitalWrite(INH_W, LOW);
        analogWrite(PWM_U, DutyCycle);
        analogWrite(PWM_V, 0);
        analogWrite(PWM_W, 0);
        break;

      case 1:
        digitalWrite(INH_U, HIGH);
        digitalWrite(INH_V, LOW);
        digitalWrite(INH_W, HIGH);
        analogWrite(PWM_U, DutyCycle);
        analogWrite(PWM_V, 0);
        analogWrite(PWM_W, 0);
        break;

      case 2:
        digitalWrite(INH_U, LOW);
        digitalWrite(INH_V, HIGH);
        digitalWrite(INH_W, HIGH);
        analogWrite(PWM_U, 0);
        analogWrite(PWM_V, DutyCycle);
        analogWrite(PWM_W, 0);
        break;

      case 3:
        digitalWrite(INH_U, HIGH);
        digitalWrite(INH_V, HIGH);
        digitalWrite(INH_W, LOW);
        analogWrite(PWM_U, 0);
        analogWrite(PWM_V, DutyCycle);
        analogWrite(PWM_W, 0);
        break;

      case 4:
        digitalWrite(INH_U, HIGH);
        digitalWrite(INH_V, LOW);
        digitalWrite(INH_W, HIGH);
        analogWrite(PWM_U, 0);
        analogWrite(PWM_V, 0);
        analogWrite(PWM_W, DutyCycle);
        break;

      case 5:
        digitalWrite(INH_U, LOW);
        digitalWrite(INH_V, HIGH);
        digitalWrite(INH_W, HIGH);
        analogWrite(PWM_U, 0);
        analogWrite(PWM_V, 0);
        analogWrite(PWM_W, DutyCycle);
        break;

      default:
        break;
    }

  }
  else {
    //CCW direction
    switch (CommutationStep) {
      case 0:
        digitalWrite(INH_U, LOW);
        digitalWrite(INH_V, HIGH);
        digitalWrite(INH_W, HIGH);
        analogWrite(PWM_U, 0);
        analogWrite(PWM_V, DutyCycle);
        analogWrite(PWM_W, 0);
        break;

      case 1:
        digitalWrite(INH_U, HIGH);
        digitalWrite(INH_V, LOW);
        digitalWrite(INH_W, HIGH);
        analogWrite(PWM_U, DutyCycle);
        analogWrite(PWM_V, 0);
        analogWrite(PWM_W, 0);
        break;

      case 2:
        digitalWrite(INH_U, HIGH);
        digitalWrite(INH_V, HIGH);
        digitalWrite(INH_W, LOW);
        analogWrite(PWM_U, DutyCycle);
        analogWrite(PWM_V, 0);
        analogWrite(PWM_W, 0);
        break;

      case 3:
        digitalWrite(INH_U, LOW);
        digitalWrite(INH_V, HIGH);
        digitalWrite(INH_W, HIGH);
        analogWrite(PWM_U, 0);
        analogWrite(PWM_V, 0);
        analogWrite(PWM_W, DutyCycle);
        break;

      case 4:
        digitalWrite(INH_U, HIGH);
        digitalWrite(INH_V, LOW);
        digitalWrite(INH_W, HIGH);
        analogWrite(PWM_U, 0);
        analogWrite(PWM_V, 0);
        analogWrite(PWM_W, DutyCycle);
        break;

      case 5:
        digitalWrite(INH_U, HIGH);
        digitalWrite(INH_V, HIGH);
        digitalWrite(INH_W, LOW);
        analogWrite(PWM_U, 0);
        analogWrite(PWM_V, DutyCycle);
        analogWrite(PWM_W, 0);
        break;

      default:
        break;
    }
  }
}

/**
   Divides a given PWM pin frequency by a divisor.

   The resulting frequency is equal to the base frequency divided by
   the given divisor:
     - Base frequencies:
        o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
        o The base frequency for pins 5 and 6 is 62500 Hz.
     - Divisors:
        o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
          256, and 1024.
        o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
          128, 256, and 1024.

   PWM frequencies are tied together in pairs of pins. If one in a
   pair is changed, the other is also changed to match:
     - Pins 5 and 6 are paired on timer0
     - Pins 9 and 10 are paired on timer1
     - Pins 3 and 11 are paired on timer2

   Note that this function will have side effects on anything else
   that uses timers:
     - Changes on pins 3, 5, 6, or 11 may cause the delay() and
       millis() functions to stop working. Other timing-related
       functions may also be affected.
     - Changes on pins 9 or 10 will cause the Servo library to function
       incorrectly.

   Thanks to macegr of the Arduino forums for his documentation of the
   PWM frequency divisors. His post can be viewed at:
     https://forum.arduino.cc/index.php?topic=16612#msg121031
*/
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


void setADCspeedFast(){

  //Source: https://forum.arduino.cc/index.php?topic=6549.0
    
  // defines for setting and clearing register bits
  #ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
  #endif
  #ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
  #endif
  
  // set prescale to 16
  sbi(ADCSRA,ADPS2) ;
  cbi(ADCSRA,ADPS1) ;
  cbi(ADCSRA,ADPS0) ;
}
