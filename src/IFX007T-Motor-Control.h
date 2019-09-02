
#ifndef IFX007T_MOTOR_CONTROL_H
#define IFX007T_MOTOR_CONTROL_H

class IFX007TMotorControl
{
    public:
                IFX007TMotorControl();
                ~IFX007TMotorControl();
        void    begin();
        void    end();
        //For example:

            //void setSpeed();
            //void configureAnyRelevantParamater();
            //... 

    private:
        //class attributes here
        //private functions here.
};

#endif /**< IFX007_*/