
#ifndef IFX007T_MOTOR_CONTROL_H
#define IFX007T_MOTOR_CONTROL_H

#include <Arduino.h>

// --------------- Define BLDC Motor parameters ------------------------------------------------------------------

//StartUp - Commutation-Counts to switch over to closed-loop
#define OpenLoopToClosedLoopCount 50

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
        void    configureBLDCMotor(uint8_t MotorPoles, uint8_t NrMagnets, bool Hallsensor, uint8_t torque);  
        void    setBLDCmotorRPMspeed(bool direction, uint16_t rpmSpeed);                         

    //------------- Help functions called by the program itself ----------------------------------------------------
        void    setPwmFrequency(uint8_t pin, uint16_t divisor);
        

    private:
        void    changeBEMFspeed(bool direction, uint16_t rpmSpeed);
        void    DoBEMFCommutation(bool dir);
        void    UpdateHardware(uint8_t CommutationStep, uint8_t Dir);       //For BLDC motor
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
        uint8_t _Torque;
        uint8_t _Commutation;
        uint16_t _lastBLDCspeed;

};

#endif /**< IFX007_*/