
#ifndef IFX007T_MOTOR_CONTROL_H
#define IFX007T_MOTOR_CONTROL_H

#include <Arduino.h>

//======= Very important ========
//#define DEBUG_IFX007T         //Uncomment, if you wish debug output or tune the motor (Disables automatic V_neutralOffset)
//===============================

#ifdef DEBUG_IFX007T
    #define DEBUG_PRINT_LN(str)  Serial.println(str)
    #define DEBUG_PRINT(str)  Serial.print(str)
    #define TRIGGER_PIN  digitalWrite(12, _debugPin); _debugPin = !_debugPin;
#else
	#define DEBUG_PRINT_LN(str)
    #define DEBUG_PRINT(str)
    #define TRIGGER_PIN
#endif

// --------------- Define row-names for the _PinAssignment - matrix -----------------------------------------------
#define INPUTPIN 0
#define INHIBITPIN 1
#define ADCPIN 2
#define REFVOLTAGE 3
// ----------------------------------------------------------------------------------------------------------------

#define TIMEOUT 500        // milliseconds

typedef struct
    {
        uint8_t MotorPolepairs;
        bool SensingMode;
        float PI_Reg_P;
        float PI_Reg_I;
        float V_neutral[4];
        float V_neutralFunct[2];        //Slope, offset
        float Phasedelay[4];
        float PhasedelayFunct[2];       //Slope, offset
    }BLDCParameter;

typedef struct
    {
        uint8_t in_U;
        uint8_t in_V;
        uint8_t in_W;
        uint8_t inh_U;
        uint8_t inh_V;
        uint8_t inh_W;
        uint8_t BEMF_U;
        uint8_t BEMF_V;
        uint8_t BEMF_W;
        uint8_t adc_Vneutral;
        uint8_t adc_IS;
        uint8_t adc_ISRC;
    }BLDCPinSetting;

//================ Class Definition ===============================================================================
class IFX007TMotorControl
{
    public:
    //------------- User Functions --------------------------------------------------------------------------------

                IFX007TMotorControl(void);
                IFX007TMotorControl(BLDCPinSetting MotorPins);
                ~IFX007TMotorControl(void);
        void    begin(void);
        void    end(void);

        void    setUniDirMotorSpeed(uint8_t motor, uint8_t dutycycle);                      //For Unidirectional motors
        void    setBiDirMotorSpeed(bool direction, uint8_t dutycycle);                      //For Bidirectional motors
        void    setBiDirMotorSpeed(bool direction, uint8_t dutycycle, uint8_t motor);       //For Bidirectional motors (overloaded function)
        
        void    configureBLDCMotor(BLDCParameter MyParameters);  
        void    setBLDCmotorRPMspeed(bool direction, uint16_t desired_rpmSpeed);
        void    setBLDCDutyCyclespeed(bool direction, uint8_t dutycycle);
        void    setHallBLDCmotorRPMspeed(bool direction, uint16_t desired_rpmSpeed, bool FieldWeakening);
        void    setHallBLDCmotorDCspeed(bool direction, uint8_t dutycycle, bool FieldWeakening); 
        uint8_t CommutateHallBLDC(uint8_t Dutycycle, bool hallsensor);                 
        void    DebugRoutine(uint8_t Serialinput);
        
    //------------- Variables ----------------------------------------------------
        uint8_t _Commutation = 0;
        BLDCParameter MotorParam;
        BLDCPinSetting MotorPins;

    private:
        bool    StartupBLDC(bool dir);                  // Algorithm to start up the motor, as long as theres no BEMF
        void    changeBEMFspeed(bool direction, uint16_t dutycycle);
        void    DoBEMFCommutation(bool dir);
        bool    DetectZeroCrossing(uint8_t Pin, bool sign);
        void    UpdateHardware(uint8_t CommutationStep);       //For BLDC motor
        void    calculateLinearFunction(float *array, float *result);
        void    setPwmFrequency(uint8_t pin, uint16_t divisor);
        void    PI_Regulator_DoWork(uint16_t desired_rpmSpeed);
        uint8_t UpdateHall(void);
        bool    WaitForCommutation(void);
        
        void    setADCspeedFast(void);

        /*  _________________________
            | INU   | INV   | INW   |
            | INHU  | INHV  | INHW  |
            | ADC_U | ADC_V | ADC_W |
            | ADC_VS| IS    | V_IS_RC
            |_______________________|
        */
        uint8_t _PinAssignment[4][3];
        uint8_t _BiDirMotorStatus=0;

        uint16_t _V_neutral;
        float _NumberofSteps;
        uint16_t _lastBLDCspeed;
        uint8_t _CurrentDutyCycle;
        bool _debugPin;
        uint16_t _Stepcounter = 0;
        uint32_t timerstart;
        uint32_t _TimeperRotation;

        // Values to start with, if debug option is turned on
        uint8_t iterations = 3;
        int16_t phasedelay = 90;
        uint8_t _V_NeutralOffset  = 100;

        // --------- For BLDC Hallsensor mode ----------

        /**
         * The first index switches between normal mode and fast mode (field weakening range)
         * The second index switches the direction between forward and backward
         * The third index for the pattern is the Hallsensor input as a dezimal value (e.g 0b00000101 is 5 dez)
         * The value at the index poition delivers the commutation state for the next step.
         */
        uint8_t HallPattern[2][2][7] = {
            {
                {9, 4,0,5,2,3,1},               /* Normal foreward */
                {9, 1,3,2,5,0,4}                /* Normal backward */
            },
            {
                {9, 5,1,0,3,4,2},               /* Fast foreward */
                {9, 0,2,1,4,5,3}                /* Fast backward */
            }
        };

        uint8_t _oldHall, _latestHall = 0;
        uint16_t _HallCounts = 0;
        unsigned long _PI_Update_Timeout = 999999999;
        float _PI_Integral = 0.0; 



};

#endif /**< IFX007_*/