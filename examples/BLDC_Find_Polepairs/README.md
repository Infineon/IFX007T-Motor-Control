# How many pole pairs does your brushless motor have?

## Hardware
Connect the Infineon BLDC motor control shield to your Arduino UNO or XMC4700 / XCM1100. 
If possible, use a power supply with current limitation, as the stepping mode used in this sketch can be very power consuming. 
If you want to use a motor with hallsensor feedback, you may first change some resistors, as the board is suited for BEMF (sensorless) mode only when delivered. So please have a look in the [Board manual](https://www.infineon.com/dgdl/Infineon-Motor_Control_Shield_with_IFX007T_for_Arduino-UserManual-v02_00-EN.pdf?fileId=5546d462694c98b401696d2026783556) on page 8 and solder up the required resistors.  

## Software
Depending on your motor, set the variable hallsenor in the top of the sketch to 1 (hallsensor used) or 0 (no hallsensor used). After uploading the sketch to your Arduino / XMC, open the serial monitor and select 115200 as baudrate. The serial output now tells you, what to do.  
The program lets the motor step the smallest possible steps in order to find out, how many steps are needed for one full revolution.  
The result has to be a multiple of 6, as you have to divide the result by 6 to get the number of pole pairs.

## Example
Your motor needs 24 steps for one revolution. 24 divided by 6 is 4. So your motor has 4 magnet pole pairs. To find out how many permanent magnets your motor has, multiply 4 * 2 = 8.
