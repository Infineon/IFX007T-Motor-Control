# Half Bridge setup

## Hardware
This example shows, how you can turn a motor in both directions.
Just take a normal DC motor.
Connect one pin to 'U' and the other pin to 'V'.
Plug in your supply voltage for the board (e.g. 12V) and run the code!

<img src="/pictures/Schematics_H-bridge.JPG" width="600">

## Software
### Functions
#### .setBiDirMotorSpeed(direction, speed)
The first argument tells the arduino in which direction it should turn (0 or 1).
The second argument tells the arduino how fast you want the motor to turn. 255 is the maximum speed, 0 stops the motor.

With the provided example code, your motor should first accelerate in one direction to full speed, brake down, and then do the same in the other direction. 

## Default Pin Assignment

|            | **U** | **V** | **W** |
|       ---|---|---|---|
|**IN**      | 11    | 10    | 9     |
|**INH**     | 6     | 5     | 3     |
|**ADC / Hall**| 17  | 16    | 15    |
|**ADC_VS** | 14 | | |