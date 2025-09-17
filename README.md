# IFX007T-Motor-Control

[![Build Checks](https://github.com/Infineon/IFX007T-Motor-Control/actions/workflows/compile_examples.yml/badge.svg?branch=devops%2Fadd-build-checks)](https://github.com/Infineon/IFX007T-Motor-Control/actions/workflows/compile_examples.yml)
[![Check links](https://github.com/Infineon/IFX007T-Motor-Control/actions/workflows/check_links.yml/badge.svg)](https://github.com/Infineon/IFX007T-Motor-Control/actions/workflows/check_links.yml)

IFX007T NovalithIC™ based DC motor control shield for Arduino.

<img src="https://github.com/Infineon/Assets/blob/master/Pictures/IFX007T_BLDC%20Shield_Pinout.jpg" width="500"> 

Library of Infineon's [BLDC motor control shield](https://www.infineon.com/cms/en/product/evaluation-boards/bldc-shield_ifx007t/) for Arduino IDE.
Refer also to the [Board manual](https://www.infineon.com/dgdl/Infineon-Motor_Control_Shield_with_IFX007T_for_Arduino-UserManual-v02_00-EN.pdf?fileId=5546d462694c98b401696d2026783556).

## Summary
The (Brushless) DC motor control shield from Infineon technologies is a high current motor control board compatible with Arduino and Infineon’s XMC4700 Boot Kit. It is equipped with three smart [IFX007T half-bridges](https://www.infineon.com/cms/en/product/power/motor-control-ics/intelligent-motor-control-ics/single-half-bridge-ics/ifx007t/#). The BLDC motor control shield is capable to drive one BLDC motor. Alternatively, it can be used to drive one or two bi-directional DC motors (H-Bridge configuration, cascaded to support second motor) or up to three uni-directional DC motors (half-bridge configuration).The implemented integrated IFX007T NovalithIC™ half-bridges can be controlled by a PWM via the IN Pin. Interfacing to a microcontroller is made easy by the integrated driver IC which features logic level inputs, diagnosis with current sense, slew rate adjustment, and dead time generation. The three IFX007T half-bridges are also fully protected against over-temperature, under-voltage, overcurrent and short circuit events.

## Key Features and Benefits
* Compatible with Arduino Uno R3
* Capable of high frequency PWM e.g. 30kHz
* Adjustable slew rates for optimized EMI by changing external resistor
* Driver circuit with logic level inputs
* Diagnosis with current sense
* Protection e.g. against over-temperature and overcurrent
* Brushed and brushless DC motor control up to 300W continuous load 
  * 8–40V nominal input voltage (optimized for 24V)
  * Average motor current 30A restricted due to PCB (IFX007T current limitation @ 55A min.)

## Supported Microcontroller Boards
|                     | Half bridge | H-bridge | BLDC BEMF | BLDC Hall |
|       ---|---|---|---|---|
|**Arduino UNO**      | yes     | yes     | yes     | yes   |
|**Aruino Mega2560**  | yes     | yes     | yes     | yes   |
|**XMC4700**          | yes     | yes     | no      | yes   |

## Usage
Please follow the example sketches in the /examples directory in this library to learn more about the usage of the library.

## PCB Design Data
In case you want to change the design or reuse it for your own projects, please find the board design for EAGLE under the following link:

[BLDC Motor Control Shield Design Data](https://www.infineon.com/dgdl/Infineon-BLDC_Motor_Control_Shield_IFX007T-PCB-v01_00-EN.zip?fileId=5546d462696dbf120169a0fb2a716f30)
