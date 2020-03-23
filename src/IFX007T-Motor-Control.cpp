/**
 * IFX007T-Motor-Control.cpp    -   Library for Arduino to control the BLDC Motor Shield with IFX007T Motor driver.
 */

#include "IFX007T-Motor-Control.h"

/**
 * Constructor
 * The optional arguments give an example pin assignment. If you wish different Pins, just pass the Pin numbers when calling the instance.
 */
IFX007TMotorControl::IFX007TMotorControl(int INHU = 6, int INHV = 5, int INHW = 3, int INU = 11, int INV = 10, int INW = 9)
{
    pinMode(INHU, OUTPUT);
    pinMode(INHV, OUTPUT);
    pinMode(INHW, OUTPUT);
    pinMode(INU, OUTPUT);
    pinMode(INV, OUTPUT);
    pinMode(INW, OUTPUT);

    digitalWrite(INHU, LOW);
    digitalWrite(INHV, HIGH);
    digitalWrite(INHW, LOW);
    digitalWrite(INU, LOW);
    digitalWrite(INV, LOW);
    digitalWrite(INV, LOW);

    _INHU = INHU;
    _INHV = INHV;
    _INHW = INHW;
    _INU = INU;
    _INV = INV;
    _INW = INW;
}

//comment the functions
IFX007TMotorControl:: ~IFX007TMotorControl(void)
{
    end();
}

/** 
 * Just some examples of class function definition
 */

void IFX007TMotorControl::begin(void)
{
    
}

void IFX007TMotorControl::end(void)
{
    
}