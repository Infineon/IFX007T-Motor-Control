# Half Bridge setup

Status: tested, works fine. 
Pictures follow.

This example shows, how you can turn a motor in both directions.
Just take a normal DC motor. Connect one pin to 'U' and the other pin to 'V'.
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
Your motor should first accelerate in one direction to full speed, brake down, and then do the same in the other direction. 