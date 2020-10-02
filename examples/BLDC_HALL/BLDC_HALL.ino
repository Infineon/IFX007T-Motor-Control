/**
 * This example code is for BLDC motor control with Hall Sensor.
 * Please be noticed it has to be modified for a diffenrent motor (e.g. motor with differnet pole number, PI-values, ...)
 * Refer to the README.md in this folder.
*/

#include "IFX007T-Motor-Control.h"
uint16_t rpmSpeed = 800;
bool direction = 0;           // 0 or 1
bool weakening = 0;           // 0=normal, 1=weak

//Create an instance of 'IFX007TMotorControl' called 'MyMotor'
IFX007TMotorControl MyMotor = IFX007TMotorControl();

void setup()
{
  Serial.begin(115200);
  Serial.println(" Infineon hall sensor BLDC motor test! ");

  MyMotor.begin();
  // Adapt the following values according to the README if necessary
  MyMotor.MotorParam.MotorPoles = 8;        // Pole pair number
  MyMotor.MotorParam.SensingMode = 1;       // If you use a Hallsensor set 1, for sensorless application 0
  
  MyMotor.configureBLDCMotor(MyMotor.MotorParam);
}

void loop()
{
  MyMotor.setHallBLDCmotorRPMspeed(direction, rpmSpeed, weakening);

  if (Serial.available() > 0)
  {
    uint8_t in = Serial.read();
    if(in == 'u') rpmSpeed += 200;          // Adapt the speed with keyboard input in the serial monitor
    if(in == 'j') rpmSpeed -= 200;
    Serial.println(rpmSpeed);
  }
  
}