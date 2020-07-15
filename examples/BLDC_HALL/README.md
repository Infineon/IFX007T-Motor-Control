# Setup for Brushlessmotor with Hallsensor

Status: not tested
Pictures follow.

## Hardware
When you have a new, unopened IFX007T board, you may first change some resistors, as the board is suited for BEMF mode only when delivered. So please have a look in the [Board manual](https://www.infineon.com/dgdl/Infineon-Motor_Control_Shield_with_IFX007T_for_Arduino-UserManual-v02_00-EN.pdf?fileId=5546d462694c98b401696d2026783556) on page 8 and solder the required resistors.

Connect the three wires of your brushless motor to the 'U V W' outputs of the board. 

## Software

### Functions
#### .setHallBLDCmotorRPMspeed(direction, rpmSpeed)
Use this function only in a continous loop without any time-critical code next to it (just like in the example sketch).
The paramter direction can be 0 or 1.
The parameter rpmSpeed can be a value from 0 to the maximum speed of your motor, e.g. 10000 RpM is already quite fast but possible. 

#### .configureBLDCMotor(MyMotor.MotorParam)
This function transmits the srtuct element 'MotorParam' to configure the behavior of your BLDC. Use it just like in the example sketch.
Edit the entry 'MotorPoles' suited to your BLDC.


## Default Pin Assignment

|            | **U** | **V** | **W** |
|       ---|---|---|---|
|**IN**      | 11    | 10    | 9     |
|**INH**     | 6     | 5     | 3     |
|**ADC / Hall**| 17  | 16    | 15    |
|**ADC_VS** | 14 | | |