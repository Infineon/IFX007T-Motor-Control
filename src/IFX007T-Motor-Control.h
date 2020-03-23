
#ifndef IFX007T_MOTOR_CONTROL_H
#define IFX007T_MOTOR_CONTROL_H

#include <Arduino.h>

class IFX007TMotorControl
{
    public:
                IFX007TMotorControl(int INHU = 6, int INHV = 5, int INHW = 3, int INU = 11, int INV = 10, int INW = 9);
                ~IFX007TMotorControl(void);
        void    begin(void);
        void    end(void);

        void    doPWM(int freq, ind dutycylce)
        //For example:

            //void setSpeed();
            //void configureAnyRelevantParamater();
            //... 

    private:
        int _INHU;
        int _INHV;
        int _INHW;
        int _INU; 
        int _INV;
        int _INW;

        //class attributes here
        //private functions here.
};

#endif /**< IFX007_*/