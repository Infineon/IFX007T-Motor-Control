
#ifndef IFX007T_MOTOR_CONTROL_H
#define IFX007T_MOTOR_CONTROL_H

#include <Arduino.h>

//#define DEBUG_IFX007T         //Uncomment, if you wish debug output

#ifdef DEBUG_IFX007T
    #define DEBUG_PRINT_LN(str)  Serial.println(str)
    #define DEBUG_PRINT(str)  Serial.print(str)
#else
	#define DEBUG_PRINT_LN(str)
    #define DEBUG_PRINT(str)
#endif

// --------------- Define row-names for the _PinAssignment - matrix -----------------------------------------------
#define InputPin 0
#define InhibitPin 1
#define AdcPin 2
#define RefVoltage 3

//================ Class Definition ===============================================================================
class IFX007TMotorControl
{
    public:

    //------------- User Functions --------------------------------------------------------------------------------

                IFX007TMotorControl(void);
                IFX007TMotorControl(uint8_t INHU, uint8_t INHV, uint8_t INHW, uint8_t INU, uint8_t INV, uint8_t INW, uint8_t ADdcU, uint8_t ADdcV, uint8_t AdcW);
                ~IFX007TMotorControl(void);
        void    begin(void);
        void    end(void);

        void    setUniDirMotorSpeed(uint8_t motor, uint8_t dutycycle);          //For Unidirectional motors; Parameters: motor can be 0, 1 or 2, dutycycle can be 0 - 255
        void    setBiDirMotorSpeed(bool direction, uint8_t dutycycle);          //For Bidirectional motors; Parametrs: direction can be 0 or 1, dutycycle can be 0 - 255
        void    configureBLDCMotor(uint8_t MotorPoles, uint8_t NrMagnets, bool Hallsensor);  
        void    setBLDCmotorRPMspeed(bool direction, uint16_t rpmSpeed);
        void    setBLDCDutyCyclespeed(bool direction, uint8_t dutycycle);                         

    //------------- Help functions called by the program itself ----------------------------------------------------
        void    setPwmFrequency(uint8_t pin, uint16_t divisor);
        
        

    private:
        bool    StartupBLDC(bool dir);                  // Algorithm to start up the motor, as long as theres no BEMF
        void    changeBEMFspeed(bool direction, uint16_t rpmSpeed);
        bool    DoBEMFCommutation(bool dir);
        void    UpdateHardware(uint8_t CommutationStep, uint8_t Dir);       //For BLDC motor
        void    DebugRoutine(void);
        void    setADCspeedFast(void);
        uint8_t gcd(uint8_t a, uint8_t b);

        /*  _________________________
            | INU   | INV   | INW   |
            | INHU  | INHV  | INHW  |
            | ADC_U | ADC_V | ADC_W |
            | ADC_VS| IS    | V_IS_RC
            |_______________________|
        */
        uint8_t _PinAssignment[4][3];


        uint32_t _V_neutral;
        uint8_t _NumberofSteps;
        uint8_t _Commutation;
        uint16_t _lastBLDCspeed;
        uint8_t _CurrentDutyCycle;
        uint8_t _TargetDutyCycle;
        bool _debugPin;

        uint8_t iterations = 3;
        int16_t phasedelay = 0;
        uint8_t _V_NeutralOffset  = 100;

};

#endif /**< IFX007_*/