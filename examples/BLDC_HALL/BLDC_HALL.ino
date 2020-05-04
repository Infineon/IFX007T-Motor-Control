//This example code is for BLDC motor control with Hall Sensor.
//Please be noticed it has to be modified for diffenrent motor (e.g. the motor with differnet pole number, reference PRM...)

#define PWM_U 11
#define PWM_V 10
#define PWM_W 9
#define EN_U 6
#define EN_V 5
#define EN_W 3
#define HALL_A A1
#define HALL_B A2
#define HALL_C A3

#define MotorPoles 8 //according the different motor it will be changed 
#define PI_REG_K 0.01
#define PI_REG_I 0.001
#define ReferenceRPM 6500.0 //the referen round per min which can set by the users 




uint8_t CommutationState = 1;
uint8_t ClosedLoop = 0;
uint8_t OpenLoopSteps = 100;
uint16_t OpenLoopDelay = 3000;
uint8_t DutyCycle = 80; // it's related to the rotate speed (round per min)
uint8_t oldHall, latestHall = 0;
uint16_t HallCounts = 0;
unsigned long PI_Update_Timeout = 999999999;
uint16_t LastRPM = 0; //the current rotate speed 

float RefRPM = ReferenceRPM;
float PI_K = PI_REG_K;
float PI_I = PI_REG_I;
float PI_Integral = 0.0; 

void setup() {

Serial.begin(115200);
setPwmFrequency(PWM_U,1); //set the frequency at 31250Hz
setPwmFrequency(PWM_V,1); //set the frequency at 31250Hz
setPwmFrequency(PWM_W,1); //set the frequency at 31250Hz
  // put your setup code here, to run once:


pinMode(EN_U, OUTPUT);
pinMode(EN_V, OUTPUT);
pinMode(EN_W, OUTPUT);
pinMode(PWM_U, OUTPUT);
pinMode(PWM_V, OUTPUT);
pinMode(PWM_W, OUTPUT);
pinMode(HALL_A, INPUT_PULLUP);
pinMode(HALL_B, INPUT_PULLUP);
pinMode(HALL_C, INPUT_PULLUP);

pinMode(13, OUTPUT);

}



void loop() {

  if (ClosedLoop==0) {
    
    delayMicroseconds(OpenLoopDelay);
    Commutate();
    UpdateHall();
    
  }
  else {
    Commutate();
    while(oldHall == UpdateHall());
    HallCounts++;
  }
  
  if (Serial.available() > 0) {
    byte in = Serial.read();
    if (in == '+') RefRPM+=100; //RefRPM + 100
    if (in == '-') RefRPM-=100; //RefRPM - 100
    //if (in == 'r') Serial.println(LastRPM,DEC); //show latest RPM status
    //if (in == 'd') Serial.println(DutyCycle,DEC); //Show DutyCycle
    //if (in == 'c') Serial.println(ClosedLoop,DEC); //Is in Closed Loop?
    //if (in == 'm') Serial.println(millis(), DEC); //TimeStamp
    //if (in == 't') Serial.println(PI_Update_Timeout, DEC); 
  }
  
  PI_Regulator_DoWork();

}



void PI_Regulator_DoWork() {
  //Do every 10 ms and in the open loop used  to accelerate and in the closed loop used to eliminate the error
  if (millis() > PI_Update_Timeout) {
    uint16_t RPM = (HallCounts * 100 * 60)/MotorPoles;
    LastRPM = RPM;
    float RPMf = (float) RPM;
    float Error = RefRPM - RPMf;
    PI_Integral = PI_Integral + Error;
    float pwm = PI_K*Error + PI_I*PI_Integral;
    //Limit PWM
    if (pwm > 200) pwm = 200;
    if (pwm < 30) pwm = 30;
    DutyCycle = (uint8_t) pwm;    
    HallCounts = 0;
    PI_Update_Timeout = millis() + 10;
  }
}


void Commutate() {
  //the circulation of the motor
  UpdateHardware (CommutationState);
  CommutationState++;
  if (CommutationState==7) CommutationState=1;
  
}


uint8_t UpdateHall() {
  // to get the new position information of the motor and compared to the older one. every 10 ms
  oldHall = latestHall;
  latestHall = (digitalRead(A3)<<2) | (digitalRead(A2)<<1) | digitalRead(A1);
  if (OpenLoopSteps > 0) {
    if (oldHall != latestHall) OpenLoopSteps--;    
    PI_Update_Timeout = millis()+10;
  }
  else {
    ClosedLoop = 1;
  }
  return latestHall;
}



void UpdateHardware(uint8_t CommutationStep) {
  //run the motor 
  switch (CommutationStep) {
    case 1:
      digitalWrite(EN_U,HIGH);
      digitalWrite(EN_V,HIGH);
      digitalWrite(EN_W,LOW);
      analogWrite(PWM_U,DutyCycle);
      analogWrite(PWM_V,0);
      analogWrite(PWM_W,0);
      break;
      
    case 2:
      digitalWrite(EN_U,HIGH);
      digitalWrite(EN_V,LOW);
      digitalWrite(EN_W,HIGH);
      analogWrite(PWM_U,DutyCycle);
      analogWrite(PWM_V,0);
      analogWrite(PWM_W,0);
      break;

   case 3:
      digitalWrite(EN_U,LOW);
      digitalWrite(EN_V,HIGH);
      digitalWrite(EN_W,HIGH);
      analogWrite(PWM_U,0);
      analogWrite(PWM_V,DutyCycle);
      analogWrite(PWM_W,0);
      break;

   case 4:
      digitalWrite(EN_U,HIGH);
      digitalWrite(EN_V,HIGH);
      digitalWrite(EN_W,LOW);
      analogWrite(PWM_U,0);
      analogWrite(PWM_V,DutyCycle);
      analogWrite(PWM_W,0);
      break;

    case 5:
      digitalWrite(EN_U,HIGH);
      digitalWrite(EN_V,LOW);
      digitalWrite(EN_W,HIGH);
      analogWrite(PWM_U,0);
      analogWrite(PWM_V,0);
      analogWrite(PWM_W,DutyCycle);
      break;

    case 6:
      digitalWrite(EN_U,LOW);
      digitalWrite(EN_V,HIGH);
      digitalWrite(EN_W,HIGH);
      analogWrite(PWM_U,0);
      analogWrite(PWM_V,0);
      analogWrite(PWM_W,DutyCycle);
      break;

   default:
   break;
  } 
}

/**
* Divides a given PWM pin frequency by a divisor.
* 
 * The resulting frequency is equal to the base frequency divided by
* the given divisor:
*   - Base frequencies:
*      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
*      o The base frequency for pins 5 and 6 is 62500 Hz.
*   - Divisors:
*      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
*        256, and 1024.
*      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
*        128, 256, and 1024.
* 
 * PWM frequencies are tied together in pairs of pins. If one in a
* pair is changed, the other is also changed to match:
*   - Pins 5 and 6 are paired on timer0
*   - Pins 9 and 10 are paired on timer1
*   - Pins 3 and 11 are paired on timer2
* 
 * Note that this function will have side effects on anything else
* that uses timers:
*   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
*     millis() functions to stop working. Other timing-related
*     functions may also be affected.
*   - Changes on pins 9 or 10 will cause the Servo library to function
*     incorrectly.
* 
 * Thanks to macegr of the Arduino forums for his documentation of the
* PWM frequency divisors. His post can be viewed at:
*   https://forum.arduino.cc/index.php?topic=16612#msg121031
*/

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
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
