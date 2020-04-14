# Half Bridge setup

Status: tested, works fine. 
Pictures follow.

This example is the easiest way to get into it: Just take a normal DC motor.
Connect one pin to 'U' and the other pin to ground.
The pin assignment between your Arduino Uno and the BLDC shield is as follows:

Inhabit pins:
INHU = 6
INHV = 5
INHW = 3

Input pins
INU = 11
INV = 10
INW = 9

Plug in your supply voltage for the board (e.g. 12V) and run the code!
Your motor should run one second with the half of his maximum speed, stop one second and do the same again.