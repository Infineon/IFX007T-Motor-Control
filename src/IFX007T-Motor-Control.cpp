/**
 * IFX007T-Motor-Control.cpp    -   Library for the Infineon BLDC Motor Control Shield with IFX007T Motor driver.
 * 
 * Tested microcontroller boards: 
 *    Arduino UNO
 *    XMC4700 Relax kit for 5V shields
 * 
 * The Library supports the following operation modes:
 *    sensorless BEMF mode (BLDC)
 *    hallsensor mode (BLDC)
 *    3 unipolar motors (DC)
 *    1 bidirectional motor (DC)
 *    2 bidirectional motors (DC)
 */

#include "IFX007T-Motor-Control.h"

/**
 * Constructor 1
 * This Constructor is called, if no arguments are given to it and thus takes the default pin assignment
 */
IFX007TMotorControl::IFX007TMotorControl(void)
{
    // ----------- Default Pin Assignment ---------------
    _PinAssignment[INPUTPIN][0] = 11;
    _PinAssignment[INPUTPIN][1] = 10;
    _PinAssignment[INPUTPIN][2] = 9;
    _PinAssignment[INHIBITPIN][0] = 6;
    _PinAssignment[INHIBITPIN][1] = 5;
    _PinAssignment[INHIBITPIN][2] = 3;
    _PinAssignment[ADCPIN][0] = A3;
    _PinAssignment[ADCPIN][1] = A2;
    _PinAssignment[ADCPIN][2] = A1;
    _PinAssignment[REFVOLTAGE][0] = A0;
    _PinAssignment[REFVOLTAGE][1] = A4;
    _PinAssignment[REFVOLTAGE][2] = A5;

}

/**
 * Constructor 2
 * This Constructor is called, if arguments are given to it.
 * Use it, if you want to use different pins of your Arduino.
 */
IFX007TMotorControl::IFX007TMotorControl(BLDCPinSetting MotorPins)
{
    _PinAssignment[INPUTPIN][0] = MotorPins.in_U;
    _PinAssignment[INPUTPIN][1] = MotorPins.in_V;
    _PinAssignment[INPUTPIN][2] = MotorPins.in_W;
    _PinAssignment[INHIBITPIN][0] = MotorPins.inh_U;
    _PinAssignment[INHIBITPIN][1] = MotorPins.inh_V;
    _PinAssignment[INHIBITPIN][2] = MotorPins.inh_W;
    _PinAssignment[ADCPIN][0] = MotorPins.BEMF_U;
    _PinAssignment[ADCPIN][1] = MotorPins.BEMF_V;
    _PinAssignment[ADCPIN][2] = MotorPins.BEMF_W;
    _PinAssignment[REFVOLTAGE][0] = MotorPins.adc_Vneutral;
    _PinAssignment[REFVOLTAGE][1] = MotorPins.adc_IS;
    _PinAssignment[REFVOLTAGE][2] = MotorPins.adc_ISRC;
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
  #ifdef DEBUG_IFX007T
    pinMode(12, OUTPUT); //for debugging
  #endif
    pinMode(_PinAssignment[INHIBITPIN][0], OUTPUT);
    pinMode(_PinAssignment[INHIBITPIN][1], OUTPUT);
    pinMode(_PinAssignment[INHIBITPIN][2], OUTPUT);
    pinMode(_PinAssignment[INPUTPIN][0], OUTPUT);
    pinMode(_PinAssignment[INPUTPIN][1], OUTPUT);
    pinMode(_PinAssignment[INPUTPIN][2], OUTPUT);

    digitalWrite(_PinAssignment[INHIBITPIN][0], LOW);
    digitalWrite(_PinAssignment[INHIBITPIN][1], LOW);
    digitalWrite(_PinAssignment[INHIBITPIN][2], LOW);
    digitalWrite(_PinAssignment[INPUTPIN][0], LOW);
    digitalWrite(_PinAssignment[INPUTPIN][1], LOW);
    digitalWrite(_PinAssignment[INPUTPIN][2], LOW);

    setPwmFrequency(_PinAssignment[INPUTPIN][0], 1);  // set Frequency to 31250 Hz
    setPwmFrequency(_PinAssignment[INPUTPIN][1], 1);
    setPwmFrequency(_PinAssignment[INPUTPIN][2], 1);

    DEBUG_PRINT_LN("Debug mode is active.");
}

void IFX007TMotorControl::end(void)
{
  /* Stop interrupts here */
  digitalWrite(_PinAssignment[INHIBITPIN][0], LOW);   // Lock the halfbridges
  digitalWrite(_PinAssignment[INHIBITPIN][1], LOW);
  digitalWrite(_PinAssignment[INHIBITPIN][2], LOW);
}

/**
 * For uniderectional motor
 * @brief Up to three motors can be controlled unidirectional
 * @param motor which output should be active, 0=U, 1=V, 2=W
 * @param dutycycle speed, can be a value from 0 - 255
 */
void IFX007TMotorControl::setUniDirMotorSpeed(uint8_t motor, uint8_t dutycycle)
{
    if(dutycycle > 0)
    {
        digitalWrite(_PinAssignment[INHIBITPIN][motor], HIGH);       //Set Inhibit to high
    }
    else                                                             //Motor is stopped
    {
        digitalWrite(_PinAssignment[INHIBITPIN][motor], LOW);
    }
    analogWrite(_PinAssignment[INPUTPIN][motor], dutycycle);
}

/**
 * For bidirectional motor
 * @brief One bidir motor can be wired to U and V, allowing one unidir motor on W.
 * @param direction can be 0 or 1
 * @param dutycycle speed, can be a value from 0 - 255
 */
void IFX007TMotorControl::setBiDirMotorSpeed(bool direction, uint8_t dutycycle)
{
    // default Pin Configuration for one Bidirectional Motor
    uint8_t pin1 = 0;   // corresponds to U
    uint8_t pin2 = 1;   // corresponds to V

    if( !(_BiDirMotorStatus & (1<<1)) )
    {
      if(dutycycle > 0)   
      {
        _BiDirMotorStatus |= (1<<0);    //on
        digitalWrite(_PinAssignment[INHIBITPIN][pin1], HIGH);
        digitalWrite(_PinAssignment[INHIBITPIN][pin2], HIGH);
      }
      else
      {
        _BiDirMotorStatus &= ~(1<<0);   //off
        digitalWrite(_PinAssignment[INHIBITPIN][pin1], LOW);
        digitalWrite(_PinAssignment[INHIBITPIN][pin2], LOW);
      }
      
      if(direction == 0)
      {
        analogWrite(_PinAssignment[INPUTPIN][pin1], dutycycle);
        digitalWrite(_PinAssignment[INPUTPIN][pin2], LOW);
      }
      else
      {
        analogWrite(_PinAssignment[INPUTPIN][pin2], dutycycle);
        digitalWrite(_PinAssignment[INPUTPIN][pin1], LOW);
      }
    }
}

/**
 * For bidirectional motor - overloaded function
 * @brief One bidir motor can be wired to U and V, allowing one unidir motor on W. If the half of possible speed is ok as well, two bidir motors are possible with common V.
 * @param motor 0: one bidir motor on U-V. 1: Two bidir motors, new dutycycle is for motor U-V. 2: Two bidir motors, new dutycycle is motor V-W.
 * @param direction can be 0 or 1
 * @param dutycycle speed, can be a value from 0 - 255
 */
void IFX007TMotorControl::setBiDirMotorSpeed(bool direction, uint8_t dutycycle, uint8_t motor)
{
    // default Pin Configuration for one Bidirectional Motor
    uint8_t pin1 = 0;   // corresponds to U
    uint8_t pin2 = 1;   // corresponds to V
    uint8_t speed =0;
    if(direction) speed = 127 - dutycycle/2;    //Calculate correct speed (motor stops with dutycycle of 127)
    else speed = 127 + dutycycle/2;

    switch(motor){
      case 0:
      if( !(_BiDirMotorStatus & (1<<1)) )
      {
        if(dutycycle > 0)   
        {
          _BiDirMotorStatus |= (1<<0);    //on
          digitalWrite(_PinAssignment[INHIBITPIN][pin1], HIGH);
          digitalWrite(_PinAssignment[INHIBITPIN][pin2], HIGH);
        }
        else
        {
          _BiDirMotorStatus &= ~(1<<0);   //off
          digitalWrite(_PinAssignment[INHIBITPIN][pin1], LOW);
          digitalWrite(_PinAssignment[INHIBITPIN][pin2], LOW);
        }
        
        if(direction == 0)
        {
          analogWrite(_PinAssignment[INPUTPIN][pin1], dutycycle);
          digitalWrite(_PinAssignment[INPUTPIN][pin2], LOW);
        }
        else
        {
          analogWrite(_PinAssignment[INPUTPIN][pin2], dutycycle);
          digitalWrite(_PinAssignment[INPUTPIN][pin1], LOW);
        }
        break;
      }
      case 1:
      if( !(_BiDirMotorStatus & (1<<0)) )   //Make sure, case 0 is not active
      {
        if(dutycycle > 0){
          _BiDirMotorStatus |= (1<<1);    //on
          digitalWrite(_PinAssignment[INHIBITPIN][0], HIGH);
          analogWrite(_PinAssignment[INPUTPIN][0], speed);
                 
          digitalWrite(_PinAssignment[INHIBITPIN][1], HIGH);  //Switch on common 50% PWM
          analogWrite(_PinAssignment[INPUTPIN][1], 127);
          
        }
        else{
          _BiDirMotorStatus &= ~(1<<1);   //off
          digitalWrite(_PinAssignment[INHIBITPIN][0], LOW);
          analogWrite(_PinAssignment[INPUTPIN][0], 0);
        }
      }
        break;
      case 2:
      if( !(_BiDirMotorStatus & (1<<0)) )    //Make sure, case 0 is not active
      {
        if(dutycycle > 0){
            _BiDirMotorStatus |= (1<<2);    //on
            digitalWrite(_PinAssignment[INHIBITPIN][2], HIGH);
            analogWrite(_PinAssignment[INPUTPIN][2], speed);
                  
            digitalWrite(_PinAssignment[INHIBITPIN][1], HIGH);  //Switch on common 50% PWM
            analogWrite(_PinAssignment[INPUTPIN][1], 127);
            
          }
          else{
            _BiDirMotorStatus &= ~(1<<2);   //off
            digitalWrite(_PinAssignment[INHIBITPIN][2], LOW);
            analogWrite(_PinAssignment[INPUTPIN][2], 0);
          }
      }
          break;
      default:
      break;
    }
    
    if(_BiDirMotorStatus == 0){               //Switch off common 50% PWM, if both Motors are off
      digitalWrite(_PinAssignment[INHIBITPIN][1], LOW);
      analogWrite(_PinAssignment[INPUTPIN][1], 0);
    }
}

/**
 * amount of teeth should be a multiple of 3
 * Numer of Magnets should be a multiple of 2
 * Hallsensor: 0 means BEMF-Mode, 1 means Hallsensor-mode
 * 
 * Refer to: https://www.allaboutcircuits.com/industry-articles/3-phase-brushless-dc-motor-control-with-hall-sensors/
 *           https://www.mikrocontroller.net/articles/Brushless-Controller_für_Modellbaumotoren
 * @param MyParameters struct element BLDC Parameters contains the edge points for V_neutral and phasedelay curves (see readme)
 */
void IFX007TMotorControl::configureBLDCMotor(BLDCParameter MyParameters)
{
    _Commutation = 0;
    _lastBLDCspeed = 0;
    _debugPin = 1;
    

    if(MyParameters.SensingMode)                        //Hall-sensor mode
    {
        #define HALLMODE
        DEBUG_PRINT_LN("Hallsensor mode");
        _CurrentDutyCycle = 80;                         // dutycycle at the beginning
        _PI_Update_Timeout = millis() + 100;
    }
    else                                                //BEMF mode
    {
        #define BEMFMODE
        DEBUG_PRINT_LN("Sensorless BEMF mode");
        _V_neutral = analogRead(_PinAssignment[REFVOLTAGE][0]);    //Dummy measurement
        calculateLinearFunction(MyParameters.V_neutral, MotorParam.V_neutralFunct);
        calculateLinearFunction(MyParameters.Phasedelay,  MotorParam.PhasedelayFunct);
        
        // Set ADC sampling time faster in order to be fast enough to detect commutation pulse:
        setADCspeedFast();
    }
    MotorParam = MyParameters;                                 //Store the parameters in a global variable
    _NumberofSteps = (float) MyParameters.MotorPolepairs * 6.0;
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
 * @param dir Direction: 0 or 1
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

  else if(_lastBLDCspeed > 1)                                   //make sure, the motor has accelerated after startup
  {
    _lastBLDCspeed = _CurrentDutyCycle;

    if(_Stepcounter > (_NumberofSteps - 1))
    {
      _TimeperRotation = micros() - _TimeperRotation;           //Calculate the passed time for one rotation
      uint16_t actual_rpmSpeed = 60000000 / _TimeperRotation;
      int16_t error = desired_rpmSpeed - actual_rpmSpeed;

        dutycycleF = _CurrentDutyCycle + 0.007 * error;         // Calculate the new dutycycle according to the proportional
      
      dutycycle = (uint16_t) (dutycycleF);

      if(abs(_CurrentDutyCycle-dutycycle) > 10){                // limit dutycycle change to 10
        if(error > 0) dutycycle = _CurrentDutyCycle + 10;
        else dutycycle = _CurrentDutyCycle - 10;
      }
      if( !(dutycycle > 30 && dutycycle < 255)) dutycycle = _lastBLDCspeed; 

      _Stepcounter = 0;
      _TimeperRotation = micros();      //Store the actual time
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

    while(!StartupBLDC(direction));
    DEBUG_PRINT_LN("Started up sensorless BLDC motor.");

    timerstart = micros();
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
      _CurrentDutyCycle = 100;                                  //Default dutycycle after startup (necessary for keyboard speed input)
    }
    else _Stepcounter++;
  }
}

/**
 * Private function
 * Before each commutation step, the reference voltage is stored in _V_neutral. 
 * _V_neutral is the middle-node voltage simulated by resistors. If now the inducted voltage on the floating wire crosses _V_neutral (zero crossing),
 * the program knows, that half of the time to commutate has passed.
 * 
 * The RPM speed is only set by the PWM duty cycle.
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
    _V_neutral = (uint16_t) (analogRead(_PinAssignment[REFVOLTAGE][0]) * (_CurrentDutyCycle/255.0)+0.5)-_V_NeutralOffset;

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
        break;
    default:
    break;
    }

    timefromzero = micros()-timerstart-phasedelay; //Calculate the passed time, since the last commutation

    delayMicroseconds(timefromzero);               // Timing is very important. When zero crossing occured, only half of the time has passed.

    //if(_Commutation == 3 || _Commutation == 0) TRIGGER_PIN;    // Toggle Debug Pin (only for debugging)
    
    UpdateHardware(_Commutation);              // Commutate: Set the new pin configuration for the next step
    timerstart= micros();                          // Store the time, when last commutation occured
    //if(_Commutation == 5 || _Commutation == 2) TRIGGER_PIN;
    
    
}

/**
 * Program function
 * @param Pin ADC Pin that should be read and be compared to V_neutral
 * @param sign 0 means BEMF voltage smaller than V_neutral, 1 means BEMF voltage larger than V_neutral
 */
bool IFX007TMotorControl::DetectZeroCrossing(uint8_t Pin, bool sign)
{
  TRIGGER_PIN;

  uint16_t BEMFvoltage = 0;
  if (sign == 0)
  {
    for (uint8_t i=0; i< iterations; i++)
    {
      BEMFvoltage = analogRead(_PinAssignment[ADCPIN][Pin]);
      if (BEMFvoltage > _V_neutral) i-=1;
    }
  }
  else
  {
    for (uint8_t i=0; i< iterations; i++)
    {
      BEMFvoltage = analogRead(_PinAssignment[ADCPIN][Pin]);
      if (BEMFvoltage < _V_neutral) i-=1;
    }
  }
  TRIGGER_PIN;

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
 * User function
 * Set the RPM speed, direction and fieldmode for hallsensor BLDCM
*/
void IFX007TMotorControl::setHallBLDCmotorRPMspeed(bool direction, uint16_t desired_rpmSpeed, bool FieldWeakening)
{
  if(desired_rpmSpeed>0)
    {
      if(_lastBLDCspeed == 0)
      {
        _oldHall = UpdateHall();
        UpdateHardware( HallPattern[FieldWeakening][direction][_oldHall] );
        _lastBLDCspeed = 1;
      }
      else
      {
        if(WaitForCommutation())
        {
          _oldHall = _latestHall;
          UpdateHardware( HallPattern[FieldWeakening][direction][_oldHall] );
        }
      }
      PI_Regulator_DoWork(desired_rpmSpeed);
      _HallCounts++;
    }
    else
    {
      end();
      _lastBLDCspeed = 0;
    } 
}

/**
 * User function
 * Set the Dutycycle speed, direction and fieldmode for hallsensor BLDCM
*/
void IFX007TMotorControl::setHallBLDCmotorDCspeed(bool direction, uint8_t dutycycle, bool FieldWeakening)
{
    _CurrentDutyCycle = dutycycle;
    if(dutycycle > 10)
    {
      if(_lastBLDCspeed == 0)
      {
        _oldHall = UpdateHall();
        UpdateHardware( HallPattern[FieldWeakening][direction][_oldHall] );
        _lastBLDCspeed = 1;
      }
      else
      {
        if(WaitForCommutation())
        {
          _oldHall = _latestHall;
          UpdateHardware( HallPattern[FieldWeakening][direction][_oldHall] );
        }
      }
    }
    else
    {
      end();
      _lastBLDCspeed = 0;
    } 
}

/**
 * For Hallsensor BLDC
 * this function is executed until either the Hallsensor pattern changes or a timer is out of time.
 * The timer is a kind of watchdog that makes sure, that the program gets not stuck, if the dutycycle was too low.
 */
bool IFX007TMotorControl::WaitForCommutation(void)
{
  uint32_t timestamp = millis();
  while(1)
  {
    if(_oldHall != UpdateHall() ) return 1;
    else if( (millis()-timestamp) > TIMEOUT)
    {
      end();
      _lastBLDCspeed = 0;
      return 0;
    }
  }
}

/**
 * For Hall sensor BLDC
 * The function will be executed every 100ms.
 * It mesaures the actual RpM speed, compares it to the desired RpM speed and regulates the CurrentDutyCycle using a PI-regulator
*/
void IFX007TMotorControl::PI_Regulator_DoWork(uint16_t desired_rpmSpeed)
{
  if (millis() > _PI_Update_Timeout)
  {
    float RPM = 0.0;
    float hallcounts = (float) _HallCounts;
    // Formula for 100ms intervall: RPM = (Hallcounts / (6 * MotorPolepairs)) * 10 * 60
    
    RPM = (_HallCounts/ _NumberofSteps) * 600;
    _PI_Update_Timeout = millis() + 100;

    float Error = desired_rpmSpeed - RPM;
    if(_CurrentDutyCycle < 240) _PI_Integral = _PI_Integral + Error;
    float pwm = MotorParam.PI_Reg_P * Error + MotorParam.PI_Reg_I * _PI_Integral;
    //Limit PWM
    if (pwm > 240) pwm = 240;
    if (pwm < 30) pwm = 30;
    _CurrentDutyCycle = (uint8_t) pwm;    
    _HallCounts = 0;
   
  }
}

/**
 * For BLDC_Find_Polepairs test program
*/
uint8_t IFX007TMotorControl::CommutateHallBLDC(uint8_t Dutycycle, bool hallsensor)
{
  _CurrentDutyCycle = Dutycycle;
   _Commutation++ ;
  if (_Commutation==6) _Commutation=0;
  UpdateHardware(_Commutation);
  if(hallsensor) return UpdateHall();
  else return 0;
}


/**
 * For hall BLDCM
 * Read the hallsensor status, to get the new position information of the motor.
 * Needs special handling for the XMC, as digitalRead() on Analog pins does only work on Arduino
 */
uint8_t IFX007TMotorControl::UpdateHall(void)
{
  _latestHall = 0; 
  
  #if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)                                           // For Arduino Boards 

  // Optimized for Arduino: max tested speed: 3300 RpM (WK = 0), 6000 RpM (WK = 1)
  _latestHall = (digitalRead(_PinAssignment[ADCPIN][0])<<2) | (digitalRead(_PinAssignment[ADCPIN][1])<<1) | digitalRead(_PinAssignment[ADCPIN][2]);

  #elif defined(XMC1100_Boot_Kit)  || defined(XMC4700_Relax_Kit)                // For XMC boards 

  //Only for XMC, max speed with Arduino only 3500 RpM
  for(uint8_t i=0; i<3; i++)
  {
    uint16_t out = analogRead(_PinAssignment[ADCPIN][i]);
    if(out>512) out = 1;
    else out = 0;
    _latestHall = _latestHall | (out<<(2-i));
  }
  #endif
  
  return _latestHall;
}

/**
 * Commutation table for brushless motors (the "6 step Process")
 * Inhibit Pin = High means active -> Inhibit Pin = Low means output is floating
*/
void IFX007TMotorControl::UpdateHardware(uint8_t CommutationStep)
{
  switch (CommutationStep) {
    case 0:
      digitalWrite(_PinAssignment[INHIBITPIN][0], HIGH);    //PWM
      digitalWrite(_PinAssignment[INHIBITPIN][1], HIGH);    //GND
      digitalWrite(_PinAssignment[INHIBITPIN][2], LOW);     //Floating
      analogWrite(_PinAssignment[INPUTPIN][0], _CurrentDutyCycle);
      analogWrite(_PinAssignment[INPUTPIN][1], 0);
      analogWrite(_PinAssignment[INPUTPIN][2], 0);
      break;

    case 1:
      digitalWrite(_PinAssignment[INHIBITPIN][0], HIGH);    //PWM
      digitalWrite(_PinAssignment[INHIBITPIN][1], LOW);     //Floating
      digitalWrite(_PinAssignment[INHIBITPIN][2], HIGH);    //GND
      analogWrite(_PinAssignment[INPUTPIN][0], _CurrentDutyCycle);
      analogWrite(_PinAssignment[INPUTPIN][1], 0);
      analogWrite(_PinAssignment[INPUTPIN][2], 0);
      break;

    case 2:
      digitalWrite(_PinAssignment[INHIBITPIN][0], LOW);     //Floating
      digitalWrite(_PinAssignment[INHIBITPIN][1], HIGH);    //PWM
      digitalWrite(_PinAssignment[INHIBITPIN][2], HIGH);    //GND
      analogWrite(_PinAssignment[INPUTPIN][0], 0);
      analogWrite(_PinAssignment[INPUTPIN][1], _CurrentDutyCycle);
      analogWrite(_PinAssignment[INPUTPIN][2], 0);
      break;

    case 3:
      digitalWrite(_PinAssignment[INHIBITPIN][0], HIGH);    //GND
      digitalWrite(_PinAssignment[INHIBITPIN][1], HIGH);    //PWM
      digitalWrite(_PinAssignment[INHIBITPIN][2], LOW);     //Floating
      analogWrite(_PinAssignment[INPUTPIN][0], 0);
      analogWrite(_PinAssignment[INPUTPIN][1], _CurrentDutyCycle);
      analogWrite(_PinAssignment[INPUTPIN][2], 0);
      break;

    case 4:
      digitalWrite(_PinAssignment[INHIBITPIN][0], HIGH);    //GND
      digitalWrite(_PinAssignment[INHIBITPIN][1], LOW);     //Floating
      digitalWrite(_PinAssignment[INHIBITPIN][2], HIGH);    //PWM
      analogWrite(_PinAssignment[INPUTPIN][0], 0);
      analogWrite(_PinAssignment[INPUTPIN][1], 0);
      analogWrite(_PinAssignment[INPUTPIN][2], _CurrentDutyCycle);
      break;

    case 5:
      digitalWrite(_PinAssignment[INHIBITPIN][0], LOW);     //Floating
      digitalWrite(_PinAssignment[INHIBITPIN][1], HIGH);    //GND
      digitalWrite(_PinAssignment[INHIBITPIN][2], HIGH);    //PWM
      analogWrite(_PinAssignment[INPUTPIN][0], 0);
      analogWrite(_PinAssignment[INPUTPIN][1], 0);
      analogWrite(_PinAssignment[INPUTPIN][2], _CurrentDutyCycle);
      break;

    default:
      break;
  }
}


#ifdef ARDUINO_AVR_UNO                                                  /** For Arduino Boards */

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

#elif ARDUINO_AVR_MEGA2560
void IFX007TMotorControl::setPwmFrequency(uint8_t pin, uint16_t divisor)
{
    byte mode;
    if (pin < 9 || pin > 10) {
      switch (divisor) {
      case 1: mode = 0x01; break;     // 31372,55 Hz  (62500,00 Hz on pin D4 & D13)
      case 8: mode = 0x02; break;     // 3921,16 Hz   (7812,50 Hz on pin D4 & D13)
      case 64: mode = 0x03; break;    // 490,20 Hz    (976,56 Hz on pin D4 & D13)
      case 256: mode = 0x04; break;   // 122,55 Hz    (244,14 Hz on pin D4 & D13)
      case 1024: mode = 0x05; break;  // 30,64 Hz     (61,04 Hz on pin D4 & D13)
      default: return;
      }
      if (pin == 4 || pin == 13) {
        TCCR0B = TCCR0B & 0b11111000 | mode;
      }
      else if (pin == 11 || pin == 12){
        TCCR1B = TCCR1B & 0b11111000 | mode;
      }
      else if (pin == 2 || pin == 3 || pin == 5){
        TCCR3B = TCCR3B & 0b11111000 | mode;
      }
      else if (pin == 6 || pin == 7 || pin == 8){
        TCCR4B = TCCR4B & 0b11111000 | mode;
      }
      else{
         TCCR5B = TCCR5B & 0b11111000 | mode;
      }
    } else if (pin == 9 || pin == 10) {
        switch (divisor) {
        case 1: mode = 0x01; break;     // 31372,55 Hz
        case 8: mode = 0x02; break;     // 3921,16 Hz
        case 32: mode = 0x03; break;    // 980,39 Hz
        case 64: mode = 0x04; break;    // 490,20 Hz
        case 128: mode = 0x05; break;   // 245,10 Hz
        case 256: mode = 0x06; break;   // 122,55 Hz
        case 1024: mode = 0x07; break;  // 30,64 Hz
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


#elif defined(XMC1100_Boot_Kit)  || defined(XMC4700_Relax_Kit)                /** For XMC boards*/ 

void IFX007TMotorControl::setPwmFrequency(uint8_t pin, uint16_t divisor)
{
  setAnalogWriteFrequency(pin, 30000);       // 31250 Hz, obviously we nee the half (fault in XMCforArduino?)
}

void IFX007TMotorControl::setADCspeedFast(void)
{

}

#endif