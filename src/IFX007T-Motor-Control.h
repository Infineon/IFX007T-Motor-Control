
#ifndef IFX007T_MOTOR_CONTROL_H
#define IFX007T_MOTOR_CONTROL_H

#include <Arduino.h>

#define InputPin 0
#define InhibitPin 1
#define HallPin 2

class IFX007TMotorControl
{
    public:

    //------------- User Functions --------------------------------------------------------------------------------

                IFX007TMotorControl(void);
                IFX007TMotorControl(uint8_t INHU, uint8_t INHV, uint8_t INHW, uint8_t INU, uint8_t INV, uint8_t INW);
                ~IFX007TMotorControl(void);
        void    begin(void);
        void    end(void);

        void    setUniDirMotorSpeed(uint8_t motor, uint8_t dutycycle);      //For Unidirectional motors; Parameters: motor can be 0, 1 or 2, dutycycle can be 0 - 255
        void    setBiDirMotorSpeed(bool direction, uint8_t dutycycle);          //For Bidirectional motors; Parametrs: direction can be 0 or 1, dutycycle can be 0 - 255

    //------------- Help functions called by the program itself ----------------------------------------------------
        void    setPwmFrequency(uint8_t pin, uint16_t divisor);
        void    setADCspeedFast(void);


    private:

        void    UpdateHardware(uint8_t CommutationStep, uint8_t Dir);       //For BLDC motor

        /*  _________________________
            | INU   | INV   | INW   |
            | INHU  | INHV  | INHW  |
            | HallU | HallV | HallW |
            |_______________________|
        */
        uint8_t _PinAssignment[3][3];

};

#endif /**< IFX007_*/