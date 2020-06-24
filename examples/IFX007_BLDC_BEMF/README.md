# Setup for BEMF Brushlessmotor

**Status:** tested on Arduino UNO and a Pichler Boost 15 with KV=1000 (some parameters may be adjusted for a different motor)

## The Theory
BEMF means **Back-Electromotive Force** and is a common way, to drive a brushless motor **without a sensor**.
Every motor has in fact three coils (can be more but they are wired together internal) and thus for each coil one wire. When driving the motor, always two wires provide the voltage, while the third wire is floating.
Once the motor spins fast enough, a voltage will be inducted in the third coil and since the corresponding wire is floating, the inducted voltage can be measured by the microcontroller. By evaluating the voltage curve, the microcontroller can conclude, at which angle the motor currently is.

## Hardware
Connect the three wires of your brushless motor to the 'U V W' outputs of the board (the order deosn't really matter, it only changes the initial direction of rotation).
Connect a voltage supply **with current limitation to 8A**, suited to the motor you want to control (typically 12V or 24V).
**Do not use a supply voltage higher than 24V, as this might damage your microcontroller board!**

<img src="/pictures/Schematics_BLDC.JPG" width="600">

In your first attempt, leave one hand on the main power switch of your BLDC shield in case of malfunction and give it a try.
It would be even better if you plug a multimeter between your power supply as well (10A fused), in order to monitor the current.

If the motor starts turning slowly, then suddenly gets boosted and keeps turning with high speed: Congratulations!

If it doesn't turn at all but makes a squeezing noise, switch off your power supply because you first have to change some values (explained below).

## Software

### Functions
#### .setBLDCDutyCyclespeed(direction, DutyCycle)
Use this function only in a continous loop without any time-critical code next to it (just like in the example sketch).
The paramter direction can be 0 or 1.
The parameter DutyCycle can be 0 to 255 but it has a bonus feature:
If you set it to 1, you can control the speed via the serial monitor, by entering 't' to increase speed or 'g' to reduce the speed.
Values till 20 will be set to 0, as this duty cycle is too low, to keep the BLDC running.

#### .configureBLDCMotor(MyMotor.MotorParam)
This function transmits the srtuct element 'MotorParam' to configure the behavior of your BLDC. Use it just like in the example sketch.

#### .DebugRoutine(Serialinput)
This function interprets the serial input. You need it if you want to control the speed or change other parameters via keyboard input. See the table *Keyboard commands for tuning* below.

### Troubleshooting
If your motor starts turning slowly for about 3 seconds but then gets stuck, you'll need to read the following.
The library controls two parameters, that depend on the current dutycycle: They are called *V_neutral offset* and *Phasedelay*.
As explained above, the program needs to detect, when it's the right moment to commutate. This is done by comparing the inducted BEMF voltage on the floating phase with a simulated neutral voltage, which should be exact the (scaled) half of your supply voltage. But as this is not precise enough, you have to subtract a value (exactly! I'm talking about *V_neutral offset*).

Now you know, when half of the time to commutate has passed, however you have to wait the same time again until you can actually commutate. Almost at least. Because as you're loosing time by setting pin values, reading serial input, etc. you have to substract a *Phasedelay* of your second time delay.

You can see it quite clear on these Oszi-screenshots:

<img src="/pictures/Oszi_V_neutralOffset.JPG" width="600">

<img src="/pictures/Oszi_Phasedelay.JPG" width="600">

However these two parameters are not constant, they are dependant of the current dutycycle (= speed), so we have to implement a function.

Lets take a look at the following charts:
<img src="https://github.com/Infineon/IFX007T-Motor-Control/blob/ardlib/pictures/explanation_parameters.jpg" width="900">

These are the values I got by tuning the parameters by hand. So every 10 dutycycle points I played araound with the values for *Phasedelay* and *V_neutral offset* (via the Serial monitor/Keyboard input), to get the minimum current but the maximum RPM speed. I recognized, you can approach the graph, when you say: At the borders you have a constant part, and in the middle there's a linear slope. The approach looks quite good for the *V_neutral offset*. Ok, maybe the *Phasedelay* graph looks a bit venturous, however it works. 
Now, what you can do, is shift the brake of slope to suit your motor. I think the picture describes it the best.

To give you an idea what current values are typical, here are mine (again for the Pichler Boost 15 BLDC motor):
<img src="https://github.com/Infineon/IFX007T-Motor-Control/blob/ardlib/pictures/diagram_current.jpg" width="600">

### Tuning
Ok, but how to find out your values? Therefore you have to change in the debug mode:
Set your Dutycycle to 1:
<img src="https://github.com/Infineon/IFX007T-Motor-Control/blob/ardlib/pictures/SetDutycycleToOne.JPG" width="300">

Uncomment the following in the *src/IFX007T_Motor-Control.h*
<img src="https://github.com/Infineon/IFX007T-Motor-Control/blob/ardlib/pictures/UncommentDegugMode.JPG" width="1000">

Upload the sketch to your Arduino.
Now you can set the *V_neutral Offset* and the *Phasedelay* just like the *Dutycycle* manually (you won't need to change the iterations, its always 3).

Here (*src/IFX007T_Motor-Control.h*) you would find the initial values after startup:
<img src="https://github.com/Infineon/IFX007T-Motor-Control/blob/ardlib/pictures/TuneStartValues.JPG" width="400">

### Keybord commands for tuning
|            | **Speed** | **V_neutralOffset** | **Phasedelay** | **Iterations** |
|       ---|---|---|---|---|
|**UP**  |t|r|e|w|
|**Down**|g|f|d|s|

Don't forget to comment out again the Debug mode after you finished playing around, otherwise your changes to the *MotorParam* won't take effect.

## Default Pin Assignment

|            | **U** | **V** | **W** |
|       ---|---|---|---|
|**IN**      | 11    | 10    | 9     |
|**INH**     | 6     | 5     | 3     |
|**ADC / Hall**| 17  | 16    | 15    |
|**ADC_VS** | 14 | | |

