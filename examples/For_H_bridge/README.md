# H-Bridge setup

## Hardware
This example shows, how you can turn a motor in both directions.
Just take a normal DC motor.
Connect one pin to 'U' and the other pin to 'V'.
Plug in your supply voltage for the board (e.g. 12V) and run the code!

<img src="/pictures/Schematics_H-bridge.JPG" width="600">

Now pay attention: You can even drive two DC motors with independent speed and independent direction! The only disadvantage is, that each motor can drive only the half of its maximum speed it would be capable of with the given supply voltage. This is because both motors share the V-connection (like you can see in the picture below). But lets assume you have a two wheel Robot with a 6V Battery, but your motors turn already fast enough with 3V, this mode would be perfect for you.

<img src="/pictures/Schematics_H-bridge_two-motors.JPG" width="600">

## Software
### Functions

#### .setBiDirMotorSpeed(motor, direction, speed)
The first argument 'motor' defines which hardware setup you have and which speed you want to control according to the following table:

|    motor      | **scenario** | **maximum speed** |
|       ---|---|---|
|**0**     | One bidirectional motor, a second unidirectional motor is possible   | Full    |
|**1**     | Two bidirectional motors, speed affects motor 1     |Half Vcc|
|**2**     | Two bidirectional motors, speed affects motor 2     |Half Vcc|

The second argument 'direction' tells the arduino in which direction it should turn (0 or 1).
The third argument 'speed' tells the arduino how fast you want the motor to turn. 255 is the maximum speed, 0 stops the motor.


With the provided example code, your motor should first accelerate in one direction to full speed, brake down, and then do the same in the other direction. 

## Default Pin Assignment

|            | **U** | **V** | **W** |
|       ---|---|---|---|
|**IN**      | 11    | 10    | 9     |
|**INH**     | 6     | 5     | 3     |
|**ADC / Hall**| 17  | 16    | 15    |
|**ADC_VS** | 14 | | |
