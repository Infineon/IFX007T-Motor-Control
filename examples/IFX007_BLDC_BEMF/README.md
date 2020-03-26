# Setup for BEMF Brushlessmotor

Status: compiling, not tested, not working
Pictures follow.

BEMF means **Back-Electromotive Force** and is a common way, to drive a brushless motor **without a sensor**. 
This is done by measuring the current deliverd by the IFX007t to detect zero-crossing.

## Hardware
Connect the three wires of your brushless motor to the 'U V W' outputs of the board (the order deosn't matter). 

## Default Pin Assignment

|            | **U** | **V** | **W** |
|       ---|---|---|---|
|**IN**      | 11    | 10    | 9     |
|**INH**     | 6     | 5     | 3     |
|**ADC / Hall**| 17  | 16    | 15    |
|**ADC_VS** | 14 | | |

## Software
To set an accourate RPM speed, you need to have the following information of your motor:
How many poles/teeth does it have (must be a multiple of 3)?
How many Magnetic Poles does it have (must be a multiple of 2)?
