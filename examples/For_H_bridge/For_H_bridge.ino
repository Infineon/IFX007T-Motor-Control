/**
 * This code is for h bridge configuration. The motor should be connected to output U and output V.
*/
#include "IFX007T-Motor-Control.h"

uint8_t speed = 0;
bool direction = 0;

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
  direction = 0;
  for(speed = 0; speed < 255; speed ++)
  {
    // First Argument: Choose which direction the Motor should turn: 0 or 1
    // Second Argument: Choose how fast it should turn: a value between 0 and 255
    MyMotor.setBiDirMotorSpeed(direction, speed);
    delay(50);
  }
  Serial.println("Brake forwards");
  for(speed = 255; speed > 0; speed --)
  {
    MyMotor.setBiDirMotorSpeed(direction, speed);
    delay(50);
  }

  Serial.println("Accelerate backwards");
  direction = 1;
  for(speed = 0; speed < 255; speed ++)
  {
    MyMotor.setBiDirMotorSpeed(direction, speed);
    delay(50);
  }
  Serial.println("Brake backwards");
  for(speed = 255; speed > 0; speed --)
  {
    MyMotor.setBiDirMotorSpeed(direction, speed);
    delay(50);
  }
}
