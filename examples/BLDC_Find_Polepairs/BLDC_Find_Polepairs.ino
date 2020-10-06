/**
 * This example code provides a test environment, to find out the number of polepairs of your BLDC motor.
*/

#include "IFX007T-Motor-Control.h"

// ----------- Change to 0 if your motor has no hallsensor -------------
#define HALLSENSOR 1
// ---------------------------------------------------------------------

uint8_t Dutycycle = 45;         // Choose a dutycycle as low as possible

uint8_t CommutationStep = 1;
uint8_t Counter = 0;
uint8_t Magnetpoles = 0;
uint8_t Magnetpolepairs = 0;
uint8_t Hallpattern = 0;
uint8_t in = 0;

//Create an instance of 'IFX007TMotorControl' called 'MyMotor'
IFX007TMotorControl MyMotor = IFX007TMotorControl();

void setup()
{
  Serial.begin(115200);
  Serial.println(" Infineon BLDC Polepairfinder ");
  Serial.println(" ");
  MyMotor.begin();
  MyMotor.MotorParam.SensingMode = HALLSENSOR;
  MyMotor.configureBLDCMotor(MyMotor.MotorParam);
}

void loop()
{
    Serial.println("Mark a point at the rotation axis of your motor in order to determine its position.");
    delay(1000);

    Serial.println("Press enter to bring motor in start position");
    while(Serial.available() == 0);
    in = Serial.read();
    Hallpattern = MyMotor.CommutateHallBLDC(Dutycycle, HALLSENSOR);     //go in initial position
    delay(100);
    MyMotor.end();

    Serial.println(" ");
    Serial.println("Press enter to start the measurement.");
    Serial.println("Press enter again to stop the measurement, when the motor did one full revolution");
    Serial.println(" ");
    while(Serial.available() == 0);
    in = Serial.read();

    Serial.println("Step  Commutation  HallpatternDEC  HallpatternBIN");
    while(Serial.available() == 0)
    {
        Counter ++;
        Hallpattern = MyMotor.CommutateHallBLDC(Dutycycle, HALLSENSOR);
        Serial.print(Counter);
        if(Counter < 10) Serial.print(" ");         //Align values
        Serial.print("          ");
        Serial.print(MyMotor._Commutation);
        Serial.print("          ");
        if(HALLSENSOR == 1)
        {
            Serial.print(Hallpattern);
            Serial.print("          ");
            Serial.println(Hallpattern, BIN);
        }
        else Serial.println(" / ");
        delay(700);
    }
    in = Serial.read();
    MyMotor.end();

    // Evaluation
    if((Counter % 2) == 1)
    {
        Serial.println("Please try again, it must be a even number, when you stop the motor");
    }
    else if((Counter % 6) > 0)
    {
        Serial.println("Please try again, it must be a multiple of 6");
    }
    else
    {
        Magnetpolepairs = Counter/6;
        Magnetpoles = Magnetpolepairs * 2;
        Serial.print("Your motor has ");
        Serial.print(Magnetpolepairs);
        Serial.print(" polepairs (equal to ");
        Serial.print(Magnetpoles);
        Serial.println(" poles)");
    }
    Serial.println("======================================================================");
    Serial.println("");
    delay(3000);
    Counter = 0;
}