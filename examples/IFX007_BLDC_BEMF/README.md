# Setup for BEMF Brushlessmotor

Status: tested on Arduino UNO, works well if Dutycyclespeed is not higher than 160, some parameters may be adjusted for a different motor

Pictures follow.


## The Theory
BEMF means **Back-Electromotive Force** and is a common way, to drive a brushless motor **without a sensor**.
Every motor has in fact three coils (can be more but they are wired together internal) and thus for each coil one wire. When driving the motor, always two wires provide the voltage, while the third wire is floating.
Once the motor spins fast enough, a voltage will be inducted in the third coil and since the corresponding wire is floating, the inducted voltage can be measured by the microcontroller. By evaluating the voltage curve, the microcontroller can conclude, at which angle the motor currently is.

## Hardware
Connect the three wires of your brushless motor to the 'U V W' outputs of the board (the order deosn't matter).
Connect a voltage supply, suited to the motor you want to control (typically 12V or 24V).
**Do not use a supply voltage higher than 24V, as this might damage your microcontroller board!**

## Software
In your first attempt, leave one hand of the main power switch of your BLDC shield in case of malfunction and give it a try.
If it starts turning slowly, then suddenly gets boosted and keeps turning with high speed: Congratulations!
If it doesn't turn at all but makes a squeezing noise, you first have to change some values, I'll explain the process here in the near future.

### .setBLDCDutyCyclespeed(direction, DutyCycle)
Use this function only in a continous loop without any time-critical code next to it (just like in the example sketch).
The paramter direction can be 0 or 1.
The parameter DutyCycle can be 0 to 255 but it has a bonus feature:
If you set it to 1, you can control the speed via the serial monitor, by entering 't' to increase speed or 'g' to reduce the speed.
Values till 20 will be set to 0, as this duty cycle is too low, to keep the BLDC running.

### Keybord commands for tuning
|            | **Speed** | **V_neutralOffset** | **Phasedelay** | **Iterations** |
|       ---|---|---|---|---|
|**UP**  |t|r|e|w|
|**Down**|g|f|d|s|


## Default Pin Assignment

|            | **U** | **V** | **W** |
|       ---|---|---|---|
|**IN**      | 11    | 10    | 9     |
|**INH**     | 6     | 5     | 3     |
|**ADC / Hall**| 17  | 16    | 15    |
|**ADC_VS** | 14 | | |

