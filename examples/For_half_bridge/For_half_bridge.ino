/**
 * This example code if for half-bridge application. The Motor should be connected to the Output U and GND
 * Default Pin configuration:
// Inhabit pins
int INHU = 6;
int INHV = 5;
int INHW = 3;

//Input pins
int INU = 11;
int INV = 10;
int INW = 9;
*/
#include "IFX007T-Motor-Control.h"

uint8_t MotorNr = 0;
uint8_t speed = 0;

//Create an instance of 'IFX007TMotorControl' called 'MyMotor'
IFX007TMotorControl MyMotor = IFX007TMotorControl();

void setup()
{

  MyMotor.begin();

}

void loop()
{
  // First Argument: Choose which Motor to drive: 0 = U, 1 = V, 2 = W
  // Second Argument: Choose how fast it should turn: a value between 0 and 255
  speed = 127;
  MyMotor.setUniDirMotorSpeed(MotorNr, speed);
  delay(1000);

  speed = 0;
  MyMotor.setUniDirMotorSpeed(MotorNr, speed);
  delay(1000);
}
