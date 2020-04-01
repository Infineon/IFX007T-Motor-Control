/**
 * IFX007T-Motor-Control.cpp    -   Library for Arduino to control the BLDC Motor Shield with IFX007T Motor driver.
 */

#include "IFX007T-Motor-Control.h"

/**
 * Constructor 1
 * This Constructor is called, if no arguments are given to it and thus takes the default pin assignment
 */
IFX007TMotorControl::IFX007TMotorControl(void)
{
    // ----------- Default Pin Assignment ---------------
    _PinAssignment[InputPin][0] = 11;
    _PinAssignment[InputPin][1] = 10;
    _PinAssignment[InputPin][2] = 9;
    _PinAssignment[InhibitPin][0] = 6;
    _PinAssignment[InhibitPin][1] = 5;
    _PinAssignment[InhibitPin][2] = 3;
    _PinAssignment[AdcPin][0] = 17;
    _PinAssignment[AdcPin][1] = 16;
    _PinAssignment[AdcPin][2] = 15;
    _PinAssignment[RefVoltage][0] = 14;
    _PinAssignment[RefVoltage][1] = 18;
    _PinAssignment[RefVoltage][2] = 19;

}

/**
 * Constructor 2
 * This Constructor is called, if arguments are given to it.
 */
IFX007TMotorControl::IFX007TMotorControl(uint8_t INHU, uint8_t INHV, uint8_t INHW, uint8_t INU, uint8_t INV, uint8_t INW, uint8_t AdcU, uint8_t AdcV, uint8_t AdcW)
{
    _PinAssignment[InputPin][0] = INU;
    _PinAssignment[InputPin][1] = INV;
    _PinAssignment[InputPin][2] =INW;
    _PinAssignment[InhibitPin][0] = INHU;
    _PinAssignment[InhibitPin][1] = INHV;
    _PinAssignment[InhibitPin][2] = INHW;
    _PinAssignment[AdcPin][0] = AdcU;
    _PinAssignment[AdcPin][1] = AdcV;
    _PinAssignment[AdcPin][2] = AdcW;
}

/**
 * Destructor
 * Make sure, the motor is stopped, bevore deleting the instance.
*/
IFX007TMotorControl:: ~IFX007TMotorControl(void)
{
    end();
}

/** 
 * Set pinmodes, ensure they are set to low and set PWM Frequeny high
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

    setPwmFrequency(_PinAssignment[InputPin][0], 1);  // set Frequency to 31250 Hz
    setPwmFrequency(_PinAssignment[InputPin][1], 1);
    setPwmFrequency(_PinAssignment[InputPin][2], 1);

    // Set ADC sampling time faster in order to be fast enough to detect commutation pulse:
    setADCspeedFast();
}

void IFX007TMotorControl::end(void)
{
  /* Stop interrupts here */
  digitalWrite(_PinAssignment[InhibitPin][0], LOW);   // Lock the halfbridges
  digitalWrite(_PinAssignment[InhibitPin][1], LOW);
  digitalWrite(_PinAssignment[InhibitPin][2], LOW);
}

void IFX007TMotorControl::setUniDirMotorSpeed(uint8_t motor, uint8_t dutycycle)
{
    if(dutycycle > 0)
    {
        digitalWrite(_PinAssignment[InhibitPin][motor], HIGH);       //Set Inhibit to high
    }
    else                                                             //Motor is stopped
    {
        digitalWrite(_PinAssignment[InhibitPin][motor], LOW);
    }
    analogWrite(_PinAssignment[InputPin][motor], dutycycle);
}

void IFX007TMotorControl::setBiDirMotorSpeed(bool direction, uint8_t dutycycle)
{
    //--------------- default Pin Configuration for the Bidirectional Motor ------------
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
 * MotorPoles: amount of teeth (should be a multiple of 3)
 * NrMagnets: Numer of Magnets (should be a multiple of 2)
 * Hallsensor: 0 means BEMF-Mode, 1 means Hallsensor-mode
 * torque: the dutycycle, the motor is driven with (a value between 0 and 255)
 * 
 * Refer to: https://www.allaboutcircuits.com/industry-articles/3-phase-brushless-dc-motor-control-with-hall-sensors/
 *           https://www.mikrocontroller.net/articles/Brushless-Controller_für_Modellbaumotoren
 */
void IFX007TMotorControl::configureBLDCMotor(uint8_t MotorPoles, uint8_t NrMagnets, bool Hallsensor, uint8_t torque)
{
    if(Hallsensor)    //Hall-sensor mode
    {
        #define HALLmode

        pinMode(_PinAssignment[AdcPin][0], INPUT_PULLUP);
        pinMode(_PinAssignment[AdcPin][1], INPUT_PULLUP);
        pinMode(_PinAssignment[AdcPin][2], INPUT_PULLUP);
    }
    else             //BEMF mode
    {
        #define BEMFmode
        _V_neutral = (((uint32_t)analogRead(_PinAssignment[RefVoltage][0]) * torque) >> 8);
    } 

    _NumberofSteps = (MotorPoles * NrMagnets) / gcd(MotorPoles, NrMagnets);
    _Torque = torque;

}

/**
 * TODO: User Function to set a new direction and RPMspeed
 */
void IFX007TMotorControl::setBLDCmotorRPMspeed(bool direction, uint16_t rpmSpeed)
{
  if(_lastBLDCspeed != rpmSpeed)
  {
    changeBEMFspeed(direction, rpmSpeed);
  }
}

/**
 * TODO: Program function: adapts the new speed to _lastBLDCspeed by accelerating the motor
 */
void IFX007TMotorControl::changeBEMFspeed(bool direction, uint16_t rpmSpeed)
{
  if(_lastBLDCspeed == 0)
  {
    uint16_t i = 5000;
    uint8_t CommStartup = 0;
    
    // Startup procedure: start rotating the field slowly and increase the speed    !experimental!
    while (i>1000) {
      delayMicroseconds(i);
      UpdateHardware(CommStartup,0);
      _Commutation = CommStartup++;
      if (CommStartup==6) CommStartup=0;
      i=i-20;
    }
  }

  // TODO: The commutation has to be controlled via interrupt!
  while(1) {
    DoBEMFCommutation(direction);
  }

  _lastBLDCspeed = rpmSpeed;
}

/**
 * Before each commutation step, the reference voltage is stored in _V_neutral. 
 * _V_neutral is the middle-node voltage simulated by resistors. If now the inducted voltage on the floating wire crosses _V_neutral (zero crossing),
 * the program knows, that half of the time to commutate has passed.
 * 
 * TODO: Add explanation! How to set accurate RPM speed? Interrupts!
 */
void IFX007TMotorControl::DoBEMFCommutation(bool dir)
{
    dir = dir*2;        // direction can be 0 or 1

    _V_neutral = (((uint32_t)analogRead(_PinAssignment[RefVoltage][0]) * _Torque) >> 8);
    //_V_neutral = analogRead(BEMF_phase);

    switch (_Commutation)
    {
    case 0:
        for (uint8_t i=0; i< 20; i++) {
        if (analogRead(_PinAssignment[AdcPin][2-dir]) > _V_neutral) i-=1;
        }
        if (analogRead(_PinAssignment[AdcPin][2-dir]) < _V_neutral) {
        _Commutation = 1;
        UpdateHardware(_Commutation,0);          
        }
        break;
    case 1:
    
        for (uint8_t i=0; i< 20; i++) {
        if (analogRead(_PinAssignment[AdcPin][1]) < _V_neutral) i-=1;
        }
        if (analogRead(_PinAssignment[AdcPin][1]) > _V_neutral) { 
        _Commutation=2;
        UpdateHardware(_Commutation,0);
        }
        break;
    case 2:
    
        for (uint8_t i=0; i< 20; i++) {
        if (analogRead(_PinAssignment[AdcPin][dir]) > _V_neutral) i-=1;
        }
        if (analogRead(_PinAssignment[AdcPin][dir]) < _V_neutral) { 
        _Commutation=3;
        UpdateHardware(_Commutation,0);
        }
        break;
    case 3:
    
        for (uint8_t i=0; i< 20; i++) {
        if (analogRead(_PinAssignment[AdcPin][2-dir]) < _V_neutral) i-=1;
        }
        if (analogRead(_PinAssignment[AdcPin][2-dir]) > _V_neutral) { 
        _Commutation=4;
        UpdateHardware(_Commutation,0);
        }
        break;
    case 4:
    
        for (uint8_t i=0; i< 20; i++) {
        if (analogRead(_PinAssignment[AdcPin][1]) > _V_neutral) i-=1;
        }
        if (analogRead(_PinAssignment[AdcPin][1]) < _V_neutral) { 
        _Commutation=5;
        UpdateHardware(_Commutation,0);
        }
        break;
    case 5:
    
        for (uint8_t i=0; i< 20; i++) {
        if (analogRead(_PinAssignment[AdcPin][dir]) < _V_neutral) i-=1;
        }
        if (analogRead(_PinAssignment[AdcPin][dir]) > _V_neutral) { 
        _Commutation=0;
        UpdateHardware(_Commutation,0);
        }
        break;
    default:
    break;
      
    }
}

/**
 * Commutation table for brushless motors
 * Inhibit Pin = High means active -> Inhibit Pin = Low means output is floating
*/
void IFX007TMotorControl::UpdateHardware(uint8_t CommutationStep, uint8_t Dir)
{
    #ifdef BEMFmode
    // update neutral voltage:
    _V_neutral = (int)(((uint32_t)analogRead(_PinAssignment[RefVoltage][0]) * _Torque) >> 8);
    #endif
 
  //CW direction
  if (Dir == 0) {

    switch (CommutationStep) {
      case 0:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);    //GND
        digitalWrite(_PinAssignment[InhibitPin][2], LOW);     //Floating
        analogWrite(_PinAssignment[InputPin][0], _Torque);
        analogWrite(_PinAssignment[InputPin][1], 0);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      case 1:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][1], LOW);     //Floating
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);    //GND
        analogWrite(_PinAssignment[InputPin][0], _Torque);
        analogWrite(_PinAssignment[InputPin][1], 0);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      case 2:
        digitalWrite(_PinAssignment[InhibitPin][0], LOW);     //Floating
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);    //GND
        analogWrite(_PinAssignment[InputPin][0], 0);
        analogWrite(_PinAssignment[InputPin][1], _Torque);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      case 3:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);    //GND
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][2], LOW);     //Floating
        analogWrite(_PinAssignment[InputPin][0], 0);
        analogWrite(_PinAssignment[InputPin][1], _Torque);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      case 4:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);    //GND
        digitalWrite(_PinAssignment[InhibitPin][1], LOW);     //Floating
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);    //PWM
        analogWrite(_PinAssignment[InputPin][0], 0);
        analogWrite(_PinAssignment[InputPin][1], 0);
        analogWrite(_PinAssignment[InputPin][2], _Torque);
        break;

      case 5:
        digitalWrite(_PinAssignment[InhibitPin][0], LOW);     //Floating
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);    //GND
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);    //PWM
        analogWrite(_PinAssignment[InputPin][0], 0);
        analogWrite(_PinAssignment[InputPin][1], 0);
        analogWrite(_PinAssignment[InputPin][2], _Torque);
        break;

      default:
        break;
    }

  }
  else {
    //CCW direction
    switch (CommutationStep) {
      case 0:
        digitalWrite(_PinAssignment[InhibitPin][0], LOW);
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);
        analogWrite(_PinAssignment[InputPin][0], 0);
        analogWrite(_PinAssignment[InputPin][1], _Torque);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      case 1:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][1], LOW);
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);
        analogWrite(_PinAssignment[InputPin][0], _Torque);
        analogWrite(_PinAssignment[InputPin][1], 0);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      case 2:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][2], LOW);
        analogWrite(_PinAssignment[InputPin][0], _Torque);
        analogWrite(_PinAssignment[InputPin][1], 0);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      case 3:
        digitalWrite(_PinAssignment[InhibitPin][0], LOW);
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);
        analogWrite(_PinAssignment[InputPin][0], 0);
        analogWrite(_PinAssignment[InputPin][1], 0);
        analogWrite(_PinAssignment[InputPin][2], _Torque);
        break;

      case 4:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][1], LOW);
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);
        analogWrite(_PinAssignment[InputPin][0], 0);
        analogWrite(_PinAssignment[InputPin][1], 0);
        analogWrite(_PinAssignment[InputPin][2], _Torque);
        break;

      case 5:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][2], LOW);
        analogWrite(_PinAssignment[InputPin][0], 0);
        analogWrite(_PinAssignment[InputPin][1], _Torque);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      default:
        break;
    }
  }
}

/**
 * TODO: Include function from an external library, as its only usefull for Arduino Platform and not for XMC.
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
     - Pins 5 and 6 are paired on timer0 (bound to delay(), millis(), micros())
     - Pins 9 and 10 are paired on timer1 (bound to Servo library)
     - Pins 3 and 11 are paired on timer2 (bound to Tone library)

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

/**
 * Source: https://forum.arduino.cc/index.php?topic=6549.0
*/
void IFX007TMotorControl::setADCspeedFast(void)
{
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

/**
 * Calculate the greatest common divisor
 * from: https://codereview.stackexchange.com/questions/66711/greatest-common-divisor
*/
uint8_t IFX007TMotorControl::gcd(uint8_t a, uint8_t b)
{
    return b == 0 ? a : gcd(b, a % b);
}