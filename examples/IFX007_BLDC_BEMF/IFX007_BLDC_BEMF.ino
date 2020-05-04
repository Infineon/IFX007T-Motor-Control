
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
uint8_t DutyCycle = 1;
bool direction = 0;


//Create an instance of 'IFX007TMotorControl' called 'MyMotor'
IFX007TMotorControl MyMotor = IFX007TMotorControl();

void setup()
{
  Serial.begin(115200);
  Serial.println(" Infineon BLDC motor test! ");

  MyMotor.begin();

  uint8_t MotorPoles = 12;              // Count the coils in your motor.
  uint8_t NumberofMagnets = 16;
  bool SensingMode = 0;                 // If you use a Hallsensor set 1 (not yet implemented), for sensorless application 0

  
  // Enter your motor specific values.
  MyMotor.configureBLDCMotor(MotorPoles, NumberofMagnets, SensingMode);

}

void loop()
{
  // First Argument: Choose which direction the motor should turn: 0 or 1
  // Second Argument: Choose how fast it should turn 0 to 255
  MyMotor.setBLDCDutyCyclespeed(direction, DutyCycle);

  if (Serial.available() > 0)
  {
    uint8_t in = Serial.read();
    MyMotor.DebugRoutine(in);           //To set speed (and Parameters) via Keyboard input. See Documentation. 
  }
}
