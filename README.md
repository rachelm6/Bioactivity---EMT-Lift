# Bioactivity---EMT-Lift

## Licensing

TODO


## About

This code is designed to control the lifting of all four legs of the Emergency Lift Assist&copy; Device (or ELA). 
It is written to run on an MSP432 but is theoretically compatible with and board in the MSP family with sufficient
I/O for the sensors and motor controllers. The Controller board used is the RB-Cyt-132, but should be compatible 
with most boards by making only minor modifications.

## System-Level Design

TODO

## Constants

In order to make the program as portable as possible, most of the functionality can be made compatible with similar 
systems by tweaking parameters. These parameters are listed below, in the order they appear in the main.c file with a 
short description.
1. ANGLE_TOLERANCE: The permissible deviation of the angle between the acceleration vector and the y axis in milliradians
2. MOTOR_STEP: Currently unused
3. PATIENT_HEIGHT: Leg height of when device is on flat ground in mm
4. DIRECTION_PINS: Logical OR of pin numbers for direction output pins for leg 1, 2, 3, 4 IN ORDER
5. UP: Boolean value of output pin which causes legHeight to increase
6. DOWN: Boolean value of output pin which causes legHeight to decrease
7. port4_mapping: Map CCRs 1,2,3,4 to pins 4.0,4.1,4.2,4.3 respectively. See [MSP432 PMAP Documentation](http://dev.ti.com/tirex/content/simplelink_msp432_sdk_1_20_00_45/docs/driverlib/msp432p4xx/html/driverlib_html/group__pmap__api.html)
	for more information.

## State Variables

These are the global variables which store the 'state' of the system. They are listed below, in the order they appear in the 
main.c file with a short description.
1. pitch: Pitch is rotation around the Z axis, positive 'backwards' or going uphill following right-hand-rule. In milliradians.
2. roll: Currently unused. Similar to pitch.
3. legHeight#: Height of corresponding leg in mm

## Functions

A description of each function, its Inputs and Outputs, and the case in which it should be used.

TODO

&copy; Bioactivity 2018