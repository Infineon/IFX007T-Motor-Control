/**
 * This example code if for half-bridge application. The Motor should be connected to the Output U and GND
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
