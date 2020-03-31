
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

// Choose which microcontroller board you have
//#define ARM_XMC
//#define AVR_Arduino

uint16_t RefRPM = 0;

//Create an instance of 'IFX007TMotorControl' called 'MyMotor'
IFX007TMotorControl MyMotor = IFX007TMotorControl();
//To change the Pins, enter your Pin configuration in the brackets.
//IFX007TMotorControl MyMotor = IFX007TMotorControl(INHU_Pin, INHV_Pin, INHW_Pin, INU_Pin, INV_Pin, INW_Pin);

void setup()
{
  Serial.begin(115200);
  Serial.println(" Infineon BLDC motor test! ");

  MyMotor.begin();

  
  // Enter your motor specific values. If you use a Hallsensor set 1, for sensorless application 0.
  // Torque means, how strong should the electormagnets pull, (0 -> nothing would happen; max 255 -> max torque)
  // configureBLDCMotor(MotorPoles, NumberofMagnets, Hallsensor, torque)
  MyMotor.configureBLDCMotor(15, 16, 0, 150);

  // First Argument: Choose which direction the motor should turn: 0 or 1
  // Second Argument: Choose how fast it should turn in Rounds per Minute (RPM): 0 to 65535
  // !! Be carefully, as high speed can damage your motor or injure persons (if its a strong motor) !!
  
  MyMotor.setBLDCmotorRPMspeed(0, 500);
}

void loop()
{
  if (Serial.available() > 0) {
    byte in = Serial.read();
    if (in == '+') RefRPM+=100;  //RefRPM + 100
    if (in == '-') RefRPM-=100;   //RefRPM - 100
    Serial.print("Actual Speed: ");
    Serial.print(RefRPM);
    Serial.println(" RPM");

    MyMotor.setBLDCmotorRPMspeed(0, RefRPM);
  }

}