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
    pinMode(12, OUTPUT); //for debugging
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

    //setPwmFrequency(_PinAssignment[InputPin][0], 1);  // set Frequency to 31250 Hz
    //setPwmFrequency(_PinAssignment[InputPin][1], 1);
    //setPwmFrequency(_PinAssignment[InputPin][2], 1);
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
void IFX007TMotorControl::configureBLDCMotor(uint8_t MotorPoles, uint8_t NrMagnets, bool Hallsensor)
{
    _Commutation = 0;
    _lastBLDCspeed = 0;
    _debugPin = 1;
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
        _V_neutral = (((uint32_t)analogRead(_PinAssignment[RefVoltage][0]) * _CurrentDutyCycle) >> 8);

        //setPwmFrequency(_PinAssignment[InhibitPin][0], 1);  // set Frequency to 31250 Hz
        //setPwmFrequency(_PinAssignment[InhibitPin][1], 1);
        //setPwmFrequency(_PinAssignment[InhibitPin][2], 1);
    }
    
    //------ Configure ADC interrupt ------------------------------------------

    uint8_t channel = 3;
    //Arduino UNO CPU Clock runs with 16MHz
    ADCSRA = 0;
    ADCSRB = 0;
    ADCSRA |= (1<<ADATE)|(1<<ADIE);                   // enable auto trigger and interrupts when measurement is complete
    //ADCSRA |= (1<<ADPS1)|(1<<ADPS0);                // 8 prescaler for 153kHz sampling rate
    //ADCSRA |= (1<<ADPS2);                           // 16 prescaler for 76,8kHz sampling rate
    ADCSRA |= (1<<ADPS2)|(1<<ADPS0);                // 32 prescaler; sampling rate: 38,4kHz; one sample lasts: 26us
    //ADCSRA |= (1<<ADPS2)|(1<<ADPS1);                // 64 prescaler; sampling rate: 19,2kHz; one sample lasts: 52us
    ADCSRB |= (0x07 & 0x00);                          //free running mode
    ADMUX |= (channel & 0x07);                        //choose ADC Port
    ADMUX |= (1<<REFS0);                               //VDD as referenz
    
    //ADCSRA |= (1<<ADEN);                             // enable ADC

    //Do a dummy-measurement to heat up the ADC
    ADCSRA |= (1<<ADSC);                              // trigger a conversion 
    //while (ADCSRA & (1<<ADSC) );                    // wait to be finished
    //(void) ADCW;
    
    _NumberofSteps = (MotorPoles * NrMagnets) / gcd(MotorPoles, NrMagnets);
}

/**
 * The Startup procedure of a BLDC motor needs a special handling, as theres no BEMF voltage, when the motor doesn't turn
 * delayMicroseconds() -> max value: 16383
 * delay() -> value in milliseconds
 */
bool IFX007TMotorControl::StartupBLDC(bool dir)
{
  _CurrentDutyCycle = 200;     //For Startup
  uint16_t i = 6000;
  /*
  while(1)
  {
    for(_Commutation = 0; _Commutation < 6; _Commutation++)
    {
      delay(i);
      UpdateHardware(_Commutation,dir);
    }
  }
  */
  
  /*
  UpdateHardware(_Commutation,dir);
  _Commutation += 1;
  delay(16);
*/

/*
 uint16_t BEMFvoltage = 0;
  while(1)
  {
    BEMFvoltage = analogRead(_PinAssignment[AdcPin][0]);
     DEBUG_PRINT("BEMFvoltage: "); DEBUG_PRINT_LN(BEMFvoltage);
     delay(50);
  }
  */
 /*
 while(1)
 {
   digitalWrite(12, _debugPin); _debugPin = !_debugPin;
   delay(5);
 }
*/

  // Startup procedure: start rotating the field slowly and increase the speed    !experimental!
  while (i>600) {
    _Commutation ++;
    if (_Commutation==6) _Commutation=0;
    delayMicroseconds(i);
    UpdateHardware(_Commutation,dir);
    i=i-20;
    //_V_neutral = (analogRead(_PinAssignment[RefVoltage][0]));
    //DEBUG_PRINT("_V_neutral: "); DEBUG_PRINT_LN(_V_neutral);
    
  }

  _lastBLDCspeed = 1;

  return 1;
}

/**
 * User function
 * TODO: User Function to set a new direction and RPMspeed
 */
void IFX007TMotorControl::setBLDCmotorRPMspeed(bool direction, uint16_t rpmSpeed)
{
  if(_lastBLDCspeed != rpmSpeed)
  {
    if(_lastBLDCspeed == 0)
    {
      while(!StartupBLDC(direction));
    }
  }
}

/**
 * User function
 * This way to control the motor speed is more easy than 'setBLDCmotorRPMspeed'.
 * The user just enters a value between 0 and 255 and thus sets the speed.
*/
void IFX007TMotorControl::setBLDCDutyCyclespeed(bool direction, uint8_t dutycycle)
{
  
  if(_lastBLDCspeed == 0)
  {
    digitalWrite(12, _debugPin); _debugPin = !_debugPin;
    while(!StartupBLDC(direction));
    DEBUG_PRINT_LN("Started up sensorless BLDC motor succesfully.");
    digitalWrite(12, _debugPin); _debugPin = !_debugPin;
  }
  //end();    //prevent the engine from short circuit (for testing only)
  
  _CurrentDutyCycle = _TargetDutyCycle = dutycycle;
  changeBEMFspeed(direction, dutycycle);
}

/**
 * Private function
 * TODO:
 * This function is called by a timer interrupt and implements a PI-regulator. It reads out the current RPM speed,
 * compares it to the desired RPM speed and adapts the PWM dutycycle.
 */
void IFX007TMotorControl::changeBEMFspeed(bool direction, uint16_t rpmSpeed)
{
    DoBEMFCommutation(direction);
  _lastBLDCspeed = rpmSpeed;
}

/**
 * CHANGED
 * Private function
 * Before each commutation step, the reference voltage is stored in _V_neutral. 
 * _V_neutral is the middle-node voltage simulated by resistors. If now the inducted voltage on the floating wire crosses _V_neutral (zero crossing),
 * the program knows, that half of the time to commutate has passed.
 * 
 * The RPM speed is only set by the PWM duty cycle. The cummutation speed will only be interrupt controlled.
 * 
 * TODO: Add explanation! Interrupts!
 */
/*
void IFX007TMotorControl::DoBEMFCommutation(bool dir)
{
    dir = dir*2;        // direction can be 0 or 1
    
    uint8_t iterations = 2;
    uint32_t timerstart;
    uint16_t timefromzero = 0;
    uint16_t lastADC = 0;
    
    digitalWrite(12, _debugPin); _debugPin = !_debugPin;
    DEBUG_PRINT("_Commutation: "); DEBUG_PRINT_LN(_Commutation);
    //_V_neutral = (((uint32_t)analogRead(_PinAssignment[RefVoltage][0]) * _CurrentDutyCycle) >> 8);
    _V_neutral = (analogRead(_PinAssignment[RefVoltage][0]));
    DEBUG_PRINT("_V_neutral: "); DEBUG_PRINT_LN(_V_neutral);

    BEMFvoltage = analogRead(_PinAssignment[AdcPin][0]);
    DEBUG_PRINT("BEMFvoltage U: "); DEBUG_PRINT_LN(BEMFvoltage);
    BEMFvoltage = analogRead(_PinAssignment[AdcPin][1]);
    DEBUG_PRINT("BEMFvoltage V: "); DEBUG_PRINT_LN(BEMFvoltage);
    BEMFvoltage = analogRead(_PinAssignment[AdcPin][2]);
    DEBUG_PRINT("BEMFvoltage W: "); DEBUG_PRINT_LN(BEMFvoltage);

    timerstart= micros();
    switch (_Commutation)
    {
    case 0:
        BEMFvoltage = analogRead(_PinAssignment[AdcPin][2-dir]);
        do{
          lastADC = BEMFvoltage;
          BEMFvoltage = analogRead(_PinAssignment[AdcPin][2-dir]);
        }while(BEMFvoltage < lastADC);
        DEBUG_PRINT("BEMFvoltage W success: "); DEBUG_PRINT_LN(BEMFvoltage);
        _Commutation = 1;
        UpdateHardware(_Commutation,0);          
        break;
    case 1:
        BEMFvoltage = analogRead(_PinAssignment[AdcPin][1]);
        do{
          lastADC = BEMFvoltage;
          BEMFvoltage = analogRead(_PinAssignment[AdcPin][1]);
        }while(BEMFvoltage > lastADC);
        DEBUG_PRINT("BEMFvoltage V success: "); DEBUG_PRINT_LN(BEMFvoltage);
        _Commutation=2;
        UpdateHardware(_Commutation,0);
        
        break;
    case 2:
    
        BEMFvoltage = analogRead(_PinAssignment[AdcPin][dir]);
        do{
          lastADC = BEMFvoltage;
          BEMFvoltage = analogRead(_PinAssignment[AdcPin][dir]);
        }while(BEMFvoltage < lastADC);
        DEBUG_PRINT("BEMFvoltage U success: "); DEBUG_PRINT_LN(BEMFvoltage);
        _Commutation=3;
        UpdateHardware(_Commutation,0);
        
        break;
    case 3:
    
        BEMFvoltage = analogRead(_PinAssignment[AdcPin][2-dir]);
        do{
          lastADC = BEMFvoltage;
          BEMFvoltage = analogRead(_PinAssignment[AdcPin][2-dir]);
        }while(BEMFvoltage > lastADC);
        DEBUG_PRINT("BEMFvoltage W success: "); DEBUG_PRINT_LN(BEMFvoltage);
        _Commutation=4;
        UpdateHardware(_Commutation,0);
        
        break;
    case 4:
        BEMFvoltage = analogRead(_PinAssignment[AdcPin][1]);
        do{
          lastADC = BEMFvoltage;
          BEMFvoltage = analogRead(_PinAssignment[AdcPin][1]);
        }while(BEMFvoltage < lastADC);
        DEBUG_PRINT("BEMFvoltage V success: "); DEBUG_PRINT_LN(BEMFvoltage);
        _Commutation=5;
        UpdateHardware(_Commutation,0);
        
        break;

    case 5:
        BEMFvoltage = analogRead(_PinAssignment[AdcPin][dir]);
        do{
          lastADC = BEMFvoltage;
          BEMFvoltage = analogRead(_PinAssignment[AdcPin][dir]);
        }while(BEMFvoltage > lastADC);
        DEBUG_PRINT("BEMFvoltage U success: "); DEBUG_PRINT_LN(BEMFvoltage);
        _Commutation=0;
        UpdateHardware(_Commutation,0);
        
        break;
    default:
    break;
      
    }
}
*/

/**
 * Private function
 * CHANGED 3
 * Interrupt controlled
 * BEMF U -> A1
 * BEMF V -> A2
 * BEMF W -> A3
 * Before each commutation step, the reference voltage is stored in _V_neutral. 
 * _V_neutral is the middle-node voltage simulated by resistors. If now the inducted voltage on the floating wire crosses _V_neutral (zero crossing),
 * the program knows, that half of the time to commutate has passed.
 * 
 * The RPM speed is only set by the PWM duty cycle. The cummutation speed will only be interrupt controlled.
 * 
 * TODO: Add explanation! Interrupts!
 */

void IFX007TMotorControl::DoBEMFCommutation(bool dir)
{
    dir = dir*2;        // direction can be 0 or 1
    uint16_t BEMFvoltage = 0;
    uint8_t iterations = 10;
    uint32_t timerstart;
    uint16_t timefromzero = 0;
    uint8_t ADCchannel = 0;
    
    digitalWrite(12, _debugPin); _debugPin = !_debugPin;
    DEBUG_PRINT("_Commutation: "); DEBUG_PRINT_LN(_Commutation);
    //_V_neutral = (((uint32_t)analogRead(_PinAssignment[RefVoltage][0]) * _CurrentDutyCycle) >> 8);
    _V_neutral = (analogRead(_PinAssignment[RefVoltage][0]));
    DEBUG_PRINT("_V_neutral: "); DEBUG_PRINT_LN(_V_neutral);

    BEMFvoltage = analogRead(_PinAssignment[AdcPin][0]);
    DEBUG_PRINT("BEMFvoltage U: "); DEBUG_PRINT_LN(BEMFvoltage);
    BEMFvoltage = analogRead(_PinAssignment[AdcPin][1]);
    DEBUG_PRINT("BEMFvoltage V: "); DEBUG_PRINT_LN(BEMFvoltage);
    BEMFvoltage = analogRead(_PinAssignment[AdcPin][2]);
    DEBUG_PRINT("BEMFvoltage W: "); DEBUG_PRINT_LN(BEMFvoltage);

    switch (_Commutation)
    {
    case 0:
        ADCchannel = 3;          
        break;
    case 1:
        ADCchannel = 2;
        break;
    case 2:
        ADCchannel = 1;
        break;
    case 3:
        ADCchannel = 3;
        break;
    case 4:
        ADCchannel = 2;
        break;
    case 5:
        ADCchannel = 1;
        break;
    default:
    ADMUX |= (ADCchannel & 0x07);
    ADCSRA |= (1<<ADEN);      //Enable ADC
    break;
    }
}


/**
 * CHANGED 2
 * umgekehrte logik (standard ist high)
 * Coummutation Table changed according to ST Application Note
 * https://www.st.com/resource/en/application_note/cd00043112-back-emf-detection-during-pwm-on-time-by-st7mc-stmicroelectronics.pdf
 * */
/*
void IFX007TMotorControl::UpdateHardware(uint8_t CommutationStep, uint8_t Dir)
{
    #ifdef BEMFmode
    // update neutral voltage:
    //_V_neutral = (int)(((uint32_t)analogRead(_PinAssignment[RefVoltage][0]) * _CurrentDutyCycle) >> 8);
    //digitalWrite(12, _debugPin); _debugPin = !_debugPin;
    //_V_neutral = (analogRead(_PinAssignment[RefVoltage][0]));
    //digitalWrite(12, _debugPin); _debugPin = !_debugPin;
    #endif
 
  //DEBUG_PRINT("Commutation State: ");
  //DEBUG_PRINT_LN(CommutationStep);
  //CW direction
  if (Dir == 0) {

    DEBUG_PRINT("CommutationStep: "); DEBUG_PRINT_LN(CommutationStep);

    switch (CommutationStep) {
      case 0:
        analogWrite(_PinAssignment[InhibitPin][0], _CurrentDutyCycle);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);                //GND - jetzt Vcc
        digitalWrite(_PinAssignment[InhibitPin][2], LOW);                 //Floating
        digitalWrite(_PinAssignment[InputPin][0], LOW);
        digitalWrite(_PinAssignment[InputPin][1], HIGH);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      case 1:
        analogWrite(_PinAssignment[InhibitPin][0], _CurrentDutyCycle);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][1], LOW);                 //Floating
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);                //GND
        digitalWrite(_PinAssignment[InputPin][0], LOW);
        analogWrite(_PinAssignment[InputPin][1], 0);
        digitalWrite(_PinAssignment[InputPin][2], HIGH);
        break;

      case 2:
        digitalWrite(_PinAssignment[InhibitPin][0], LOW);                 //Floating
        analogWrite(_PinAssignment[InhibitPin][1], _CurrentDutyCycle);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);                //GND
        analogWrite(_PinAssignment[InputPin][0], 0);
        digitalWrite(_PinAssignment[InputPin][1], LOW);
        digitalWrite(_PinAssignment[InputPin][2], HIGH);
        break;

      case 3:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);                //GND
        analogWrite(_PinAssignment[InhibitPin][1], _CurrentDutyCycle);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][2], LOW);                 //Floating
        digitalWrite(_PinAssignment[InputPin][0], HIGH);
        digitalWrite(_PinAssignment[InputPin][1], LOW);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      case 4:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);                //GND
        digitalWrite(_PinAssignment[InhibitPin][1], LOW);                 //Floating
        analogWrite(_PinAssignment[InhibitPin][2], _CurrentDutyCycle);    //PWM
        digitalWrite(_PinAssignment[InputPin][0], HIGH);
        analogWrite(_PinAssignment[InputPin][1], 0);
        digitalWrite(_PinAssignment[InputPin][2], LOW);
        break;

      case 5:
        digitalWrite(_PinAssignment[InhibitPin][0], LOW);                 //Floating
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);                //GND
        analogWrite(_PinAssignment[InhibitPin][2], _CurrentDutyCycle);    //PWM
        analogWrite(_PinAssignment[InputPin][0], 0);
        digitalWrite(_PinAssignment[InputPin][1], HIGH);
        digitalWrite(_PinAssignment[InputPin][2], LOW);
        break;

      default:
        break;
    }

  }
  else {
    //CCW direction
    switch (CommutationStep) {
      case 0:
        digitalWrite(_PinAssignment[InhibitPin][0], LOW);                 //Floating
        analogWrite(_PinAssignment[InhibitPin][1], _CurrentDutyCycle);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);                //GND
        analogWrite(_PinAssignment[InputPin][0], 0);
        digitalWrite(_PinAssignment[InputPin][1], HIGH);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      case 1:
        analogWrite(_PinAssignment[InhibitPin][0], _CurrentDutyCycle);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][1], LOW);                 //Floating
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);                //GND
        digitalWrite(_PinAssignment[InputPin][0], HIGH);
        analogWrite(_PinAssignment[InputPin][1], 0);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      case 2:
        analogWrite(_PinAssignment[InhibitPin][0], _CurrentDutyCycle);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);                //GND
        digitalWrite(_PinAssignment[InhibitPin][2], LOW);                 //Floating
        digitalWrite(_PinAssignment[InputPin][0], HIGH);
        analogWrite(_PinAssignment[InputPin][1], 0);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      case 3:
        digitalWrite(_PinAssignment[InhibitPin][0], LOW);                 //Floating
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);                //GND
        analogWrite(_PinAssignment[InhibitPin][2], _CurrentDutyCycle);    //PWM
        analogWrite(_PinAssignment[InputPin][0], 0);
        analogWrite(_PinAssignment[InputPin][1], 0);
        digitalWrite(_PinAssignment[InputPin][2], HIGH);
        break;

      case 4:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);                //GND
        digitalWrite(_PinAssignment[InhibitPin][1], LOW);                 //Floating
        analogWrite(_PinAssignment[InhibitPin][2], _CurrentDutyCycle);    //PWM
        analogWrite(_PinAssignment[InputPin][0], 0);
        analogWrite(_PinAssignment[InputPin][1], 0);
        digitalWrite(_PinAssignment[InputPin][2], HIGH);
        break;

      case 5:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);                //GND
        analogWrite(_PinAssignment[InhibitPin][1], _CurrentDutyCycle);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][2], LOW);                 //Floating
        analogWrite(_PinAssignment[InputPin][0], 0);
        digitalWrite(_PinAssignment[InputPin][1], HIGH);
        analogWrite(_PinAssignment[InputPin][2], 0);
        break;

      default:
        break;
    }
  }
}
*/

/**
 * CHANGED 3
 * Disable the use of PWM (only high and low) -> speed controlled by anlaog input voltage
 * Commutation table for brushless motors (the "6 step Process")
 * Inhibit Pin = High means active -> Inhibit Pin = Low means output is floating
*/

void IFX007TMotorControl::UpdateHardware(uint8_t CommutationStep, uint8_t Dir)
{
    #ifdef BEMFmode
    // update neutral voltage:
    //_V_neutral = (int)(((uint32_t)analogRead(_PinAssignment[RefVoltage][0]) * _CurrentDutyCycle) >> 8);
    //digitalWrite(12, _debugPin); _debugPin = !_debugPin;
    //_V_neutral = (analogRead(_PinAssignment[RefVoltage][0]));
    //digitalWrite(12, _debugPin); _debugPin = !_debugPin;
    #endif
 
  //CW direction
  if (Dir == 0) {

    switch (CommutationStep) {
      case 0:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);    //GND
        digitalWrite(_PinAssignment[InhibitPin][2], LOW);     //Floating
        digitalWrite(_PinAssignment[InputPin][0], HIGH);
        digitalWrite(_PinAssignment[InputPin][1], LOW);
        digitalWrite(_PinAssignment[InputPin][2], LOW);
        break;

      case 1:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][1], LOW);     //Floating
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);    //GND
        digitalWrite(_PinAssignment[InputPin][0], HIGH);
        digitalWrite(_PinAssignment[InputPin][1], LOW);
        digitalWrite(_PinAssignment[InputPin][2], LOW);
        break;

      case 2:
        digitalWrite(_PinAssignment[InhibitPin][0], LOW);     //Floating
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);    //GND
        digitalWrite(_PinAssignment[InputPin][0], LOW);
        digitalWrite(_PinAssignment[InputPin][1], HIGH);
        digitalWrite(_PinAssignment[InputPin][2], LOW);
        break;

      case 3:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);    //GND
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);    //PWM
        digitalWrite(_PinAssignment[InhibitPin][2], LOW);     //Floating
        digitalWrite(_PinAssignment[InputPin][0], LOW);
        digitalWrite(_PinAssignment[InputPin][1], HIGH);
        digitalWrite(_PinAssignment[InputPin][2], LOW);
        break;

      case 4:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);    //GND
        digitalWrite(_PinAssignment[InhibitPin][1], LOW);     //Floating
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);    //PWM
        digitalWrite(_PinAssignment[InputPin][0], LOW);
        digitalWrite(_PinAssignment[InputPin][1], LOW);
        digitalWrite(_PinAssignment[InputPin][2], HIGH);
        break;

      case 5:
        digitalWrite(_PinAssignment[InhibitPin][0], LOW);     //Floating
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);    //GND
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);    //PWM
        digitalWrite(_PinAssignment[InputPin][0], LOW);
        digitalWrite(_PinAssignment[InputPin][1], LOW);
        digitalWrite(_PinAssignment[InputPin][2], HIGH);
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
        digitalWrite(_PinAssignment[InputPin][0], LOW);
        digitalWrite(_PinAssignment[InputPin][1], HIGH);
        digitalWrite(_PinAssignment[InputPin][2], LOW);
        break;

      case 1:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][1], LOW);
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);
        digitalWrite(_PinAssignment[InputPin][0], HIGH);
        digitalWrite(_PinAssignment[InputPin][1], LOW);
        digitalWrite(_PinAssignment[InputPin][2], LOW);
        break;

      case 2:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][2], LOW);
        digitalWrite(_PinAssignment[InputPin][0], HIGH);
        digitalWrite(_PinAssignment[InputPin][1], LOW);
        digitalWrite(_PinAssignment[InputPin][2], LOW);
        break;

      case 3:
        digitalWrite(_PinAssignment[InhibitPin][0], LOW);
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);
        digitalWrite(_PinAssignment[InputPin][0], LOW);
        digitalWrite(_PinAssignment[InputPin][1], LOW);
        digitalWrite(_PinAssignment[InputPin][2], HIGH);
        break;

      case 4:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][1], LOW);
        digitalWrite(_PinAssignment[InhibitPin][2], HIGH);
        digitalWrite(_PinAssignment[InputPin][0], LOW);
        digitalWrite(_PinAssignment[InputPin][1], LOW);
        digitalWrite(_PinAssignment[InputPin][2], HIGH);
        break;

      case 5:
        digitalWrite(_PinAssignment[InhibitPin][0], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][1], HIGH);
        digitalWrite(_PinAssignment[InhibitPin][2], LOW);
        digitalWrite(_PinAssignment[InputPin][0], LOW);
        digitalWrite(_PinAssignment[InputPin][1], HIGH);
        digitalWrite(_PinAssignment[InputPin][2], LOW);
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
 * Calculate the greatest common divisor
 * from: https://codereview.stackexchange.com/questions/66711/greatest-common-divisor
*/
uint8_t IFX007TMotorControl::gcd(uint8_t a, uint8_t b)
{
    return b == 0 ? a : gcd(b, a % b);
}


ISR (ADC_vect)
{
  uint16_t adc_data;
  adc_data = ADC;
  if(_Commutation == 0 || _Commutation == 2 || _Commutation ==4)
  {
    if (adc_data < _V_neutral)
    {
      _Commutation += 1;
      if(_Commutation == 6) _Commutation = 0;
      IFX007TMotorControl.UpdateHardware(_Commutation, 0);
      ADCSRA = ADCSRA & 0b01111111 | (0<<ADEN);      //Disable ADC
      //Start timer
    }
  }
  if(_Commutation == 1 || _Commutation == 3 || _Commutation == 5)
  {
    if (adc_data < _V_neutral)
    {
      _Commutation += 1;
      IFX007TMotorControl.UpdateHardware(_Commutation, 0);
      ADCSRA = ADCSRA & 0b01111111 | (0<<ADEN);      //Disable ADC
      //Start timer
    }
  }
}
