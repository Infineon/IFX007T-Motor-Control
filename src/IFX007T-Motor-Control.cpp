/**
 * IFX007T-Motor-Control.cpp    -   Library for Arduino to control the BLDC Motor Shield with IFX007T Motor driver.
 */

#include "IFX007T-Motor-Control.h"

/**
 * Constructor
 * The optional arguments give an example pin assignment. If you wish different Pins, just pass the Pin numbers when calling the instance.
 */
IFX007TMotorControl::IFX007TMotorControl(void)
{
    uint8_t _PinAssignment[3][3] =
    {
        {11, 10, 9},
        {6, 5, 3},
        {}
    };
}

//(uint8_t INHU = 6, uint8_t INHV = 5, uint8_t INHW = 3, uint8_t INU = 11, uint8_t INV = 10, uint8_t INW = 9)
IFX007TMotorControl::IFX007TMotorControl(uint8_t INHU, uint8_t INHV, uint8_t INHW, uint8_t INU, uint8_t INV, uint8_t INW)
{
    uint8_t _PinAssignment[3][3] =
    {
        {INU, INV, INW},
        {INHU, INHV, INHW},
        {}
    };
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
    pinMode(_PinAssignment[InhibitPin][0], OUTPUT);
    pinMode(_PinAssignment[InhibitPin][1], OUTPUT);
    pinMode(_PinAssignment[InhibitPin][2], OUTPUT);
    pinMode(_PinAssignment[InputPin][0], OUTPUT);
    pinMode(_PinAssignment[InputPin][1], OUTPUT);
    pinMode(_PinAssignment[InputPin][2], OUTPUT);

    digitalWrite(_PinAssignment[InhibitPin][0], LOW);
    digitalWrite(_PinAssignment[InhibitPin][1], LOW);
    digitalWrite(_PinAssignment[InhibitPin][2], LOW);
    digitalWrite(_PinAssignment[InputPin][0], LOW);
    digitalWrite(_PinAssignment[InputPin][1], LOW);
    digitalWrite(_PinAssignment[InputPin][2], LOW);

    setPwmFrequency(_PinAssignment[InputPin][0], 1);
    setPwmFrequency(_PinAssignment[InputPin][1], 1);
    setPwmFrequency(_PinAssignment[InputPin][2], 1);
}

void IFX007TMotorControl::end(void)
{
    
}

void IFX007TMotorControl::setUniDirMotorSpeed(uint8_t motor, uint8_t dutycycle)
{
    if(dutycycle > 0)
    {
        digitalWrite(_PinAssignment[InhibitPin][motor], HIGH);       //Set Inhibit to high
    }
    else                                //Motor is stopped
    {
        digitalWrite(_PinAssignment[InhibitPin][motor], LOW);
    }
    analogWrite(_PinAssignment[InputPin][motor], dutycycle);
}

void IFX007TMotorControl::setBiDirMotorSpeed(bool direction, uint8_t dutycycle)
{
    //--------------- default Pin Configuration dor the Bidirectional Motor ------------
    uint8_t pin1 = 0;   // corresponds tu U   
    uint8_t pin2 = 1;   // corresponds to V

    if(dutycycle > 0)   
    {
        digitalWrite(_PinAssignment[InhibitPin][pin1], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][pin2], HIGH);
    }
    else
    {
        digitalWrite(_PinAssignment[InhibitPin][pin1], LOW);
        digitalWrite(_PinAssignment[InhibitPin][pin2], LOW);
    }
    
    if(direction == 0)
    {
        analogWrite(_PinAssignment[InputPin][pin1], dutycycle);
        digitalWrite(_PinAssignment[InputPin][pin2], LOW);
    }
    else
    {
        analogWrite(_PinAssignment[InputPin][pin2], dutycycle);
        digitalWrite(_PinAssignment[InputPin][pin1], LOW);
    }
}

/**
   Divides a given PWM pin frequency by a divisor.

   The resulting frequency is equal to the base frequency divided by
   the given divisor:
     - Base frequencies:
        o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
        o The base frequency for pins 5 and 6 is 62500 Hz.
     - Divisors:
        o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
          256, and 1024.
        o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
          128, 256, and 1024.

   PWM frequencies are tied together in pairs of pins. If one in a
   pair is changed, the other is also changed to match:
     - Pins 5 and 6 are paired on timer0
     - Pins 9 and 10 are paired on timer1
     - Pins 3 and 11 are paired on timer2

   Note that this function will have side effects on anything else
   that uses timers:
     - Changes on pins 3, 5, 6, or 11 may cause the delay() and
       millis() functions to stop working. Other timing-related
       functions may also be affected.
     - Changes on pins 9 or 10 will cause the Servo library to function
       incorrectly.

   Thanks to macegr of the Arduino forums for his documentation of the
   PWM frequency divisors. His post can be viewed at:
     https://forum.arduino.cc/index.php?topic=16612#msg121031
*/
void IFX007TMotorControl::setPwmFrequency(uint8_t pin, uint16_t divisor)
{
    byte mode;
    if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
        switch (divisor) {
        case 1: mode = 0x01; break;
        case 8: mode = 0x02; break;
        case 64: mode = 0x03; break;
        case 256: mode = 0x04; break;
        case 1024: mode = 0x05; break;
        default: return;
        }
        if (pin == 5 || pin == 6) {
        TCCR0B = TCCR0B & 0b11111000 | mode;
        } else {
        TCCR1B = TCCR1B & 0b11111000 | mode;
        }
    } else if (pin == 3 || pin == 11) {
        switch (divisor) {
        case 1: mode = 0x01; break;
        case 8: mode = 0x02; break;
        case 32: mode = 0x03; break;
        case 64: mode = 0x04; break;
        case 128: mode = 0x05; break;
        case 256: mode = 0x06; break;
        case 1024: mode = 0x07; break;
        default: return;
        }
        TCCR2B = TCCR2B & 0b11111000 | mode;
    }
}


void IFX007TMotorControl::setADCspeedFast(void)
{
    //Source: https://forum.arduino.cc/index.php?topic=6549.0
    
    // defines for setting and clearing register bits
    #ifndef cbi
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
    #endif
    #ifndef sbi
    #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
    #endif
    
    // set prescale to 16
    sbi(ADCSRA,ADPS2) ;
    cbi(ADCSRA,ADPS1) ;
    cbi(ADCSRA,ADPS0) ;
}