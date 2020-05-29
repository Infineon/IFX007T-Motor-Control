/**
 * IFX007T-Motor-Control.cpp    -   Library for Arduino to control the BLDC Motor Shield with IFX007T Motor driver.
 * 
 * TODO:
 * Implement RPM Function for sensorless BLDC
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
IFX007TMotorControl::IFX007TMotorControl(BLDCPinSetting MotorPins)
{
    _PinAssignment[InputPin][0] = MotorPins.in_U;
    _PinAssignment[InputPin][1] = MotorPins.in_V;
    _PinAssignment[InputPin][2] = MotorPins.in_W;
    _PinAssignment[InhibitPin][0] = MotorPins.inh_U;
    _PinAssignment[InhibitPin][1] = MotorPins.inh_V;
    _PinAssignment[InhibitPin][2] = MotorPins.inh_W;
    _PinAssignment[AdcPin][0] = MotorPins.BEMF_U;
    _PinAssignment[AdcPin][1] = MotorPins.BEMF_V;
    _PinAssignment[AdcPin][2] = MotorPins.BEMF_W;
    _PinAssignment[RefVoltage][0] = MotorPins.adc_Vneutral;
    _PinAssignment[RefVoltage][1] = MotorPins.adc_IS;
    _PinAssignment[RefVoltage][2] = MotorPins.adc_ISRC;
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
 * 
 * Refer to: https://www.allaboutcircuits.com/industry-articles/3-phase-brushless-dc-motor-control-with-hall-sensors/
 *           https://www.mikrocontroller.net/articles/Brushless-Controller_für_Modellbaumotoren
 */
void IFX007TMotorControl::configureBLDCMotor(BLDCParameter MyParameters)
{
    _Commutation = 0;
    _lastBLDCspeed = 0;
    _debugPin = 1;
    

    if(MyParameters.SensingMode)    //Hall-sensor mode (Not supported yet)
    {
        #define HALLmode
        pinMode(_PinAssignment[AdcPin][0], INPUT_PULLUP);
        pinMode(_PinAssignment[AdcPin][1], INPUT_PULLUP);
        pinMode(_PinAssignment[AdcPin][2], INPUT_PULLUP);
    }
    else             //BEMF mode
    {
        #define BEMFmode
        _V_neutral = analogRead(_PinAssignment[RefVoltage][0]);    //Dummy measurement
        MotorParam = MyParameters;                                 //Store the parameters in a global variable
        calculateLinearFunction(MyParameters.V_neutral, MotorParam.V_neutralFunct);
        calculateLinearFunction(MyParameters.Phasedelay,  MotorParam.PhasedelayFunct);
    }
    //_NumberofSteps = (MotorPoles * NrMagnets) / gcd(MotorPoles, NrMagnets);
    _NumberofSteps = ((MyParameters.MotorPoles /2 +1) * 6);    //experimaental, need exact formula
}


/**
 * Program function
 * Calculates the slope and offset out of two given points
 */
void IFX007TMotorControl::calculateLinearFunction(float *array, float *result)
{
  result[0] = (array[3]-array[1])/(array[2]-array[0]);                  //slope
  result[1] = array[1]-(result[0] * array[0]);                    //offset
}


/**
 * Program function
 * The Startup procedure of a BLDC motor needs a special handling, as theres no BEMF voltage, when the motor doesn't turn
 * delayMicroseconds() -> max possible value: 16383
 * delay() -> value in milliseconds
 */
bool IFX007TMotorControl::StartupBLDC(bool dir)
{
  _CurrentDutyCycle = 150;     //Initial Speed for Startup
  uint16_t i = 7000;           //Delay to start with


  while (i>1000)
  {
    if(dir == 0)
    {
      _Commutation ++;
      if (_Commutation==6) _Commutation=0;
    }
    else
    {
      if (_Commutation==0) _Commutation=6;
      _Commutation --;
    }
    delayMicroseconds(i);
    UpdateHardware(_Commutation);
    i=i-30;                     // Decrease the delay, maybe you have to play araound with this value
  }


/*
  for(uint8_t i = 0; i < 42; i++)
  {
    _Commutation ++;
    if (_Commutation==6) _Commutation=0;

    delay(500); 
    UpdateHardware(_Commutation);
  }
  while(1);
*/

  _lastBLDCspeed = 1;
  return 1;
}

/**
 * User function
 * @param direction can be 0 or 1 (forward or backward)
 * @param desired_rpmSpeed can be a value between 0 and Vcc*KV (the KV mentioned on your motor)
 */
void IFX007TMotorControl::setBLDCmotorRPMspeed(bool direction, uint16_t desired_rpmSpeed)
{
  float dutycycleF;
  uint16_t dutycycle;

  if(_lastBLDCspeed == 0)
  {
    while(!StartupBLDC(direction));
    DEBUG_PRINT_LN("Started up sensorless BLDC motor.");
  }

  else if(_lastBLDCspeed > 1)                             //make sure, the motor has accelerated after startup
  {
    _lastBLDCspeed = _CurrentDutyCycle;

    if(_Stepcounter > (_NumberofSteps - 1))
    {
      _TimeperRotation = micros() - _TimeperRotation;     //Calculate the passed time for one rotation
      uint16_t actual_rpmSpeed = 60000000 / _TimeperRotation;
      int16_t error = desired_rpmSpeed - actual_rpmSpeed;
    
      _RPM_Integral = _RPM_Integral + error;

      if(_RPM_Integral < -10000) _RPM_Integral = -10000;
      else if(_RPM_Integral > 10000) _RPM_Integral = 10000;

      //if(error > 99)
      //{
        dutycycleF = _CurrentDutyCycle + 0.003 * error + 0.0001 * _RPM_Integral; // Calculate the new dutycycle according to the PI
        /*
      }
      else
      {
        dutycycleF = _CurrentDutyCycle + 0.01 * error + 0.001 * _RPM_Integral; // Calculate the new dutycycle according to the PI
      }
      */

      dutycycle = (uint16_t) (dutycycleF);
      if( !(dutycycle > 20 && dutycycle < 255)) dutycycle = _lastBLDCspeed;    // Limit the dutycycle

      _Stepcounter = 0;
      _TimeperRotation = micros();      //Store the actual time
      //Serial.println(dutycycle);
    }
    else _Stepcounter ++;
  }

  changeBEMFspeed(direction, dutycycle);
}

/**
 * User function
 * This way to control the motor speed is more easy than 'setBLDCmotorRPMspeed'.
 * The user just enters a value between 0 and 255 and thus sets the speed.
 * @param direction can be 0 or 1 (forward or backward)
 * @param dutycycle can be a value between 0 and 255 to set the speed.
 *                  dutycycle = 1 has a special behaviour, as this enables the speed control via the keyboard input.
*/
void IFX007TMotorControl::setBLDCDutyCyclespeed(bool direction, uint8_t dutycycle)
{
  
  if(_lastBLDCspeed == 0)
  {
    TRIGGER_PIN;
    while(!StartupBLDC(direction));
    DEBUG_PRINT_LN("Started up sensorless BLDC motor.");
    TRIGGER_PIN;
  }
  
  //_CurrentDutyCycle = _TargetDutyCycle = dutycycle;
  changeBEMFspeed(direction, dutycycle);
}

/**
 * Private function
 * This function supervises the behavior of the current duty cycle and makes shure that the moter accelerates to the desired dutycycle after it has been started up.
 * Therefore we have the global _lastBLDCspeed variable, which has three possible states:
 * _lastBLDC = 0:   the motor is turned of
 * _lastBLDC = 1:   the motor has passed the startup algorithm
 * _lastBlDC = 2:   the motor has accelerated to the desired dutycycle and runs stable in BEMF mode
 */
void IFX007TMotorControl::changeBEMFspeed(bool direction, uint16_t dutycycle)
{
  DoBEMFCommutation(direction);
  if (_lastBLDCspeed > 1 )
  {
    if(dutycycle != 1) _CurrentDutyCycle = dutycycle;           // dutycycle = 1 means keyboard controlled dutycycle
    if(dutycycle > 1 && dutycycle < 20) _CurrentDutyCycle = 0;  // Values < 20 make no sense, as its to less to keep the motor runnning
  }

  else
  {
    if((_Stepcounter) > 1000)
    {
      _lastBLDCspeed = 2;
      _CurrentDutyCycle = 100;
    }
    else _Stepcounter++;
  }

  /*
  if((_Stepcounter) > 500)
  {
    if(_lastBLDCspeed < 2){
      _lastBLDCspeed = 2;
      _CurrentDutyCycle = 100;                                   //Default dutycycle after startup (necessary for keyboard speed input)
    }
    if(dutycycle != 1) _CurrentDutyCycle = dutycycle;           // dutycycle = 1 means keyboard controlled dutycycle
    if(dutycycle > 1 && dutycycle < 20) _CurrentDutyCycle = 0;  // Values < 20 make no sense, as its to less to keep the motor runnning
  }
  else _Stepcounter++;
  */
}

/**
 * Private function
 * Before each commutation step, the reference voltage is stored in _V_neutral. 
 * _V_neutral is the middle-node voltage simulated by resistors. If now the inducted voltage on the floating wire crosses _V_neutral (zero crossing),
 * the program knows, that half of the time to commutate has passed.
 * 
 * The RPM speed is only set by the PWM duty cycle.
 * 
 * TODO: DIRECTION; DEBUGROUTINE; RPMSPEED
 */

void IFX007TMotorControl::DoBEMFCommutation(bool direction)
{
  uint8_t dir;
  uint16_t timefromzero = 0;

  if (direction == 0 || direction == 1)   // make sure that direction can only be 0 or 1
  {
    dir = direction*2;                      
  }
  else dir = direction = 0;           //default direction, when wrong input
    
 
    #ifndef DEBUG_IFX007T
      if((_lastBLDCspeed > 1) && (_lastBLDCspeed != _CurrentDutyCycle))   // Update only, when motor has accelerated and dutycycle changed
      {
        //Calculate the Offset voltage according to my measurements (its linear and proportional to Dutycycle)
        _V_NeutralOffset = (uint8_t) (_CurrentDutyCycle * MotorParam.V_neutralFunct[0] + MotorParam.V_neutralFunct[1]);
        if(_V_NeutralOffset > MotorParam.V_neutral[1]) _V_NeutralOffset = MotorParam.V_neutral[1];
        if(_V_NeutralOffset < MotorParam.V_neutral[3]) _V_NeutralOffset = MotorParam.V_neutral[3];

        phasedelay = (uint8_t) (_CurrentDutyCycle * MotorParam.PhasedelayFunct[0] + MotorParam.PhasedelayFunct[1]);
        if(phasedelay > MotorParam.Phasedelay[1]) phasedelay = 120;
        if(phasedelay < MotorParam.Phasedelay[3]) phasedelay = MotorParam.Phasedelay[3];

        _lastBLDCspeed = _CurrentDutyCycle;
      }

    #endif

    //Calculate the reference voltage and store in the global _V_neutral
    _V_neutral = (uint16_t) (analogRead(_PinAssignment[RefVoltage][0]) * (_CurrentDutyCycle/255.0)+0.5)-_V_NeutralOffset;

    switch (_Commutation)
    {
    case 0:
        while(!DetectZeroCrossing(2, direction));   //wait till zerocrossing occured
        _Commutation = 1+2*dir;     
        break;
    case 1:
        while(!DetectZeroCrossing(1, 1-direction));
        _Commutation=2-dir;
        break;
    case 2:
        while(!DetectZeroCrossing(0, direction));
        _Commutation=3-dir;
        TRIGGER_PIN; 

        break;
    case 3:
        while(!DetectZeroCrossing(2, 1-direction));
        _Commutation=4-dir;
        break;
    case 4:
        while(!DetectZeroCrossing(1, direction));
        _Commutation=5-dir;
        break;
    case 5:
        while(!DetectZeroCrossing(0, 1-direction));
        _Commutation=0+2*dir;
        TRIGGER_PIN;
        break;
    default:
    break;
    }

    timefromzero = micros()-timerstart-phasedelay; //Calculate the passed time, since the last commutation

    delayMicroseconds(timefromzero);               // Timing is very important. When zero crossing occured, only half of the time has passed.

    if(_Commutation == 3 || _Commutation == 0) TRIGGER_PIN;    // Toggle Debug Pin (only for debugging)
    
    UpdateHardware(_Commutation);              // Commutate: Set the new pin configuration for the next step
    timerstart= micros();                          // Store the time, when last commutation occured
    if(_Commutation == 5 || _Commutation == 2) TRIGGER_PIN;
    
    
}

/**
 * Program function
 * @param Pin ADC Pin that should be read and be compared to V_neutral
 * @param sign 0 means BEMF voltage smaller than V_neutral, 1 means BEMF voltage bigger than V_neutral
 */
bool IFX007TMotorControl::DetectZeroCrossing(uint8_t Pin, bool sign)
{
  uint16_t BEMFvoltage = 0;
  if (sign == 0)
  {
    for (uint8_t i=0; i< iterations; i++)
    {
      BEMFvoltage = analogRead(_PinAssignment[AdcPin][Pin]);
      if (BEMFvoltage > _V_neutral) i-=1;
    }
  }
  else
  {
    for (uint8_t i=0; i< iterations; i++)
    {
      BEMFvoltage = analogRead(_PinAssignment[AdcPin][Pin]);
      if (BEMFvoltage < _V_neutral) i-=1;
    }
  }
  return 1;
}


/**
 * User function
 * The function should be called in the main loop and input parameter is the inbut of the Serial monitor.
 * It is used to set the Dutycycle speed by keyboard commands and adapt the Motor specific parameters in Debug mode.
 * Commands are (Up,Down):
 * iterations: w,s
 * phasedelay: e,d
 * _V_NeutralOffset: r,f
 * _CurrentDutycycle_ t,g
 */
void IFX007TMotorControl::DebugRoutine(uint8_t Serialinput)
{
  #ifdef DEBUG_IFX007T
    if (Serialinput == 'w')
    {
      iterations+=1;
      Serial.print("iterations: ");
      Serial.println(iterations);
    }
    if (Serialinput == 's' && iterations > 1){
      iterations-=1;
      Serial.print("iterations: ");
      Serial.println(iterations);
    }
    if (Serialinput == 'e'){
      phasedelay+=3;
      Serial.print("phasedelay (us): ");
      Serial.println(phasedelay);
    }
    if (Serialinput == 'd'){
      phasedelay-=3;
      Serial.print("phasedelay (us): ");
      Serial.println(phasedelay);
    }
    if (Serialinput == 'r'){
      _V_NeutralOffset+=2;
      Serial.print("_V_NeutralOffset: ");
      Serial.println(_V_NeutralOffset);
    }
    if (Serialinput == 'f'){
      _V_NeutralOffset-=2;
      Serial.print("_V_NeutralOffset: ");
      Serial.println(_V_NeutralOffset);
    }
  #endif

    if (Serialinput == 't' && _CurrentDutyCycle < 255){
      _CurrentDutyCycle+=5;
      Serial.print("DC: ");
      Serial.println(_CurrentDutyCycle);
    }
    if (Serialinput == 'g' && _CurrentDutyCycle > 0){
      _CurrentDutyCycle-=5;
      Serial.print("DC: ");
      Serial.println(_CurrentDutyCycle);
    }
}

/**
 * Commutation table for brushless motors (the "6 step Process")
 * Inhibit Pin = High means active -> Inhibit Pin = Low means output is floating
*/
void IFX007TMotorControl::UpdateHardware(uint8_t CommutationStep)
{
  switch (CommutationStep) {
    case 0:
      digitalWrite(_PinAssignment[InhibitPin][0], HIGH);    //PWM
      digitalWrite(_PinAssignment[InhibitPin][1], HIGH);    //GND
      digitalWrite(_PinAssignment[InhibitPin][2], LOW);     //Floating
      analogWrite(_PinAssignment[InputPin][0], _CurrentDutyCycle);
      analogWrite(_PinAssignment[InputPin][1], 0);
      analogWrite(_PinAssignment[InputPin][2], 0);
      break;

    case 1:
      digitalWrite(_PinAssignment[InhibitPin][0], HIGH);    //PWM
      digitalWrite(_PinAssignment[InhibitPin][1], LOW);     //Floating
      digitalWrite(_PinAssignment[InhibitPin][2], HIGH);    //GND
      analogWrite(_PinAssignment[InputPin][0], _CurrentDutyCycle);
      analogWrite(_PinAssignment[InputPin][1], 0);
      analogWrite(_PinAssignment[InputPin][2], 0);
      break;

    case 2:
      digitalWrite(_PinAssignment[InhibitPin][0], LOW);     //Floating
      digitalWrite(_PinAssignment[InhibitPin][1], HIGH);    //PWM
      digitalWrite(_PinAssignment[InhibitPin][2], HIGH);    //GND
      analogWrite(_PinAssignment[InputPin][0], 0);
      analogWrite(_PinAssignment[InputPin][1], _CurrentDutyCycle);
      analogWrite(_PinAssignment[InputPin][2], 0);
      break;

    case 3:
      digitalWrite(_PinAssignment[InhibitPin][0], HIGH);    //GND
      digitalWrite(_PinAssignment[InhibitPin][1], HIGH);    //PWM
      digitalWrite(_PinAssignment[InhibitPin][2], LOW);     //Floating
      analogWrite(_PinAssignment[InputPin][0], 0);
      analogWrite(_PinAssignment[InputPin][1], _CurrentDutyCycle);
      analogWrite(_PinAssignment[InputPin][2], 0);
      break;

    case 4:
      digitalWrite(_PinAssignment[InhibitPin][0], HIGH);    //GND
      digitalWrite(_PinAssignment[InhibitPin][1], LOW);     //Floating
      digitalWrite(_PinAssignment[InhibitPin][2], HIGH);    //PWM
      analogWrite(_PinAssignment[InputPin][0], 0);
      analogWrite(_PinAssignment[InputPin][1], 0);
      analogWrite(_PinAssignment[InputPin][2], _CurrentDutyCycle);
      break;

    case 5:
      digitalWrite(_PinAssignment[InhibitPin][0], LOW);     //Floating
      digitalWrite(_PinAssignment[InhibitPin][1], HIGH);    //GND
      digitalWrite(_PinAssignment[InhibitPin][2], HIGH);    //PWM
      analogWrite(_PinAssignment[InputPin][0], 0);
      analogWrite(_PinAssignment[InputPin][1], 0);
      analogWrite(_PinAssignment[InputPin][2], _CurrentDutyCycle);
      break;

    default:
      break;
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