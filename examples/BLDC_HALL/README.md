# Setup for Brushlessmotor with Hallsensor

**Status**: Tested on Arduino UNO with a RS-PRO hall BLDCM

## Hardware
When you have a new, unopened IFX007T board, you may first change some resistors, as the board is suited for BEMF (sensorless) mode only when delivered. So please have a look in the [Board manual](https://www.infineon.com/dgdl/Infineon-Motor_Control_Shield_with_IFX007T_for_Arduino-UserManual-v02_00-EN.pdf?fileId=5546d462694c98b401696d2026783556) on page 8 and solder up the required resistors.  

<img src="/pictures/Schematics_BLDC_hall.JPG" width="600">

Connect the three phase wires of your brushless motor to the 'U V W' outputs of the board. Connect the 5 wires of your hall encoder to the green terminal. Refer to the datasheet of your BLDC to find out the correct order and colors of your wires.
**Do not use a supply voltage higher than 40V, as this might damage your microcontroller board!**

## Software

### Functions
#### .setHallBLDCmotorRPMspeed(direction, rpmSpeed, FieldWeakening)
This function sets the RpM speed for a hallsensor based BLDC motor.
Use this function only in a continous loop without any time-critical code next to it (just like in the example sketch).  
The paramter direction can be 0 or 1.  
The parameter rpmSpeed can be a value from 0 to the maximum speed of your motor.  
The paramter FieldWeakening can be 0 or 1. 0 claims 'normal' mode, with 1 the motor is driven in the field weakening range, means the motor turns with its double speed but with less torque.  
This function uses an integrated PI-regulator, to keep the desired speed. 

#### .setHallBLDCmotorDCspeed(direction, DCSpeed, FieldWeakening)
This function sets the dutycyclespeed for a hallsensor based BLDC motor.
Use this function only in a continous loop without any time-critical code next to it (just like in the example sketch).  
The paramter direction can be 0 or 1.  
The parameter DCSpeed can be a value from 0 (motor stop) to 255 (full speed).  
The paramter FieldWeakening can be 0 or 1. 0 claims 'normal' mode, with 1 the motor is driven in the field weakening range, means the motor turns with its double speed but with less torque.  

#### .configureBLDCMotor(MyMotor.MotorParam)
This function transmits the srtuct element 'MotorParam' to configure the behavior of your BLDC. Use it just like in the example sketch.
Edit the entry 'MotorPolepairs' suited to your BLDC.  


## Default Pin Assignment

|            | **U** | **V** | **W** |
|       ---|---|---|---|
|**IN**      | 11    | 10    | 9     |
|**INH**     | 6     | 5     | 3     |
|**ADC / Hall**| 17  | 16    | 15    |
|**ADC_VS** | 14 | | |
