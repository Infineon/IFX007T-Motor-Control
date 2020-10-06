
/**
 * This example code if for sensorless brushless motor application. The motor should be connected to the outputs U V and W.
 * Refer to the README.md in this folder.
 * 
*/

#include "IFX007T-Motor-Control.h"
uint8_t DutyCycle = 1;        //0-255   1 for keyboard input
bool direction = 0;           // 0 or 1

//Create an instance of 'IFX007TMotorControl' called 'MyMotor'
IFX007TMotorControl MyMotor = IFX007TMotorControl();

void setup()
{
  Serial.begin(115200);
  Serial.println(" Infineon BLDC motor test! ");

  MyMotor.begin();
  // Adapt the following values according to the README if necessary
  MyMotor.MotorParam.MotorPolepairs = 4;    // Amount of polepairs. If your motor has 8 poles, it has 4 pole PAIRS) 
  MyMotor.MotorParam.SensingMode = 0;       // If you use a Hallsensor set 1, for sensorless application 0
  
  MyMotor.MotorParam.V_neutral[0] = 160;    // High dutycycle, when V_neutral is const
  MyMotor.MotorParam.V_neutral[1] = 120;    // Value of high V_neutral
  MyMotor.MotorParam.V_neutral[2] = 50;     // Low dutycycle, when V_neutral is const 
  MyMotor.MotorParam.V_neutral[3] = 46;     // Value of low V_neutral
  MyMotor.MotorParam.Phasedelay[0] = 200;   // High dutycycle, when phasedelay is const
  MyMotor.MotorParam.Phasedelay[1] = 123;   // Value of high phasedelay
  MyMotor.MotorParam.Phasedelay[2] = 120;   // Low dutycycle, when phasedelay is const
  MyMotor.MotorParam.Phasedelay[3] = 80;    // Value of low phasedelay

  MyMotor.configureBLDCMotor(MyMotor.MotorParam);
}

void loop()
{
  MyMotor.setBLDCDutyCyclespeed(direction, DutyCycle);
  if (Serial.available() > 0)
  {
    uint8_t in = Serial.read();
    MyMotor.DebugRoutine(in);           //To set speed (and Parameters) via Keyboard input. See Documentation. 
  }
 
}
