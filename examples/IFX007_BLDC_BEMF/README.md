# Setup for BEMF Brushlessmotor

Status: tested, works under specific conditions, some parameters may be adjusted for a different motor, explanation follows.

Pictures follow.


## The Theory
BEMF means **Back-Electromotive Force** and is a common way, to drive a brushless motor **without a sensor**.
Every motor has in fact three coils (can be more but they are wired together internal) and thus for each coil one wire. When driving the motor, always two wires provide the voltage, while the third wire is floating.
Once the motor spins fast enough, a voltage will be inducted in the third coil and since the corresponding wire is floating, the inducted voltage can be measured by the microcontroller. By evaluating the voltage curve, the microcontroller can conclude, at which angle the motor currently is.

## Hardware
Connect the three wires of your brushless motor to the 'U V W' outputs of the board (the order deosn't matter).
Connect a voltage supply, suited to the motor you want to control (typically 12V or 24V).
**Do not use a supply voltage higher than 24V, as this might damage your microcontroller board!**

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
