# Half Bridge setup

## Hardware
This example is the easiest way to control up to three motors in one predefined direction with changeable speed.
Just take a normal DC motor.
Connect one pin to 'U' and the other pin to ground.
Plug in your supply voltage for the board (e.g. 12V) and run the code!

<img src="/pictures/Schematics_half-bridge.JPG" width="600">

## Software
### Functions
#### .setUniDirMotorSpeed(MotorNr, speed)
The first argument tells which motor you want to control (U=0, V=1, W=2).
The second argument tells the arduino how fast you want the motor to turn. 255 is the maximum speed, 0 stops the motor.


With the provided example code, your motor should run one second with the half of his maximum speed, stop one second and do the same again.

## Default Pin Assignment

|            | **U** | **V** | **W** |
|       ---|---|---|---|
|**IN**      | 11    | 10    | 9     |
|**INH**     | 6     | 5     | 3     |
|**ADC / Hall**| 17  | 16    | 15    |
|**ADC_VS** | 14 | | |