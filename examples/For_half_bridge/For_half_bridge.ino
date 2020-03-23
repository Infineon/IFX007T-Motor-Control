/**
 * This example code if for half-bridge application. The Motor should be connected to the Output V and GND/VS
*/

// Inhabit pins
int INHU = 6;
int INHV = 5;
int INHW = 3;

//Input pins
int INU = 11;
int INV = 10;
int INW = 9;

#include "IFX007T-Motor-Control.h"

//Create an instance of 'IFX007TMotorControl' called 'MyMotor'
IFX007TMotorControl MyMotor = IFX007TMotorControl(INHU, INHV, INHW, INU, INV, INW);

void setup()
{
}

void loop()
{
//  Output V with PWM
//  Can adjust the frequency and dutycycle by changing delay time.
    digitalWrite(INV, LOW);
    delayMicroseconds(25);
    digitalWrite(INV, HIGH);
    delayMicroseconds(25);
}
