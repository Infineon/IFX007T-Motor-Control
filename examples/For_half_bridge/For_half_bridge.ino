/**
 * This example code if for half-bridge application. The Motor should be connected to the Output V and GND/VS
 * 
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

//Create an instance of 'IFX007TMotorControl' called 'MyMotor'
IFX007TMotorControl MyMotor = IFX007TMotorControl();
//To change the Pins, enter your Pin configuration in the brackets.
//IFX007TMotorControl MyMotor = IFX007TMotorControl(INHU_Pin, INHV_Pin, INHW_Pin, INU_Pin, INV_Pin, INW_Pin);

void setup()
{
  MyMotor.begin();


  // First Argument: Choose which Motor to drive: 0 = U, 1 = V, 2 = W
  // Second Argument: Choose how fast it should turn: a value between 0 and 255
  
  MyMotor.setUniDirMotorSpeed(1, 127);
}

void loop()
{

}
