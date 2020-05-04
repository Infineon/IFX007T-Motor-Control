/**
 * This code is for h bridge configuration. The motor should be connected to output U and output V.
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

uint8_t steps = 0;

//Create an instance of 'IFX007TMotorControl' called 'MyMotor'
IFX007TMotorControl MyMotor = IFX007TMotorControl();

void setup()
{
  Serial.begin(9600);
  Serial.println(" Infineon Bidirectional DC motor test! ");

  MyMotor.begin();
}

void loop()
{
  Serial.println("Accelerate forwards");
  for(steps = 0; steps < 255; steps ++)
  {
    // setBiDirMotorSpeed(direction, speed);
    // First Argument: Choose which direction the Motor should turn: 0 or 1
    // Second Argument: Choose how fast it should turn: a value between 0 and 255
    MyMotor.setBiDirMotorSpeed(1, steps);
    delay(50);
  }
  Serial.println("Brake forwards");
  for(steps = 255; steps > 0; steps --)
  {
    MyMotor.setBiDirMotorSpeed(1, steps);
    delay(50);
  }
  Serial.println("Accelerate backwards");
  for(steps = 0; steps < 255; steps ++)
  {
    MyMotor.setBiDirMotorSpeed(0, steps);
    delay(50);
  }
  Serial.println("Brake backwards");
  for(steps = 255; steps > 0; steps --)
  {
    MyMotor.setBiDirMotorSpeed(0, steps);
    delay(50);
  }
}
