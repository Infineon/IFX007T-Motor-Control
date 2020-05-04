
/**
 * This example code if for sensorless brushless motor application. The motor should be connected to the outputs U V and W.
 * 
Default Pin configuration:
// Inhabit pins
INHU = 6
INHV = 5
INHW = 3

//Input pins
iINU = 11
INV = 10
INW = 9

// Based on ADC phase voltage inputs:
BEMF_U = A3
BEMF_V = A2
BEMF_W = A1
ADC_VS = A0
*/

#include "IFX007T-Motor-Control.h"
byte DutyCycle = 1;


//Create an instance of 'IFX007TMotorControl' called 'MyMotor'
IFX007TMotorControl MyMotor = IFX007TMotorControl();

void setup()
{
  Serial.begin(115200);
  Serial.println(" Infineon BLDC motor test! ");

  MyMotor.begin();
  
  // Enter your motor specific values. If you use a Hallsensor set 1, for sensorless application 0.
  // configureBLDCMotor(MotorPoles, NumberofMagnets, Hallsensor)
  MyMotor.configureBLDCMotor(15, 16, 0);

  // First Argument: Choose which direction the motor should turn: 0 or 1
  // Second Argument: Choose how fast it should turn 0 to 255
  // !! Be carefully, as high speed can damage your motor or injure persons (if its a strong motor) !!
}

void loop()
{
  MyMotor.setBLDCDutyCyclespeed(0, DutyCycle);
  if (Serial.available() > 0)
  {
    byte in = Serial.read();
    MyMotor.DebugRoutine(in);
  }
}
