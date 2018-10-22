/*
MSP controller for EMT Lift System
X is to the front, Y is down, Z is to the right
Assuming that no Roll is accounted for. Still measured but no action will be taken on it


    I____________I
 1 []            [] 2
    |            |              X   Z <-------------
    |            |              |
    |            |              |
    |            |              |
    |            |              |
    |            |              \/
 4 []            [] 3

*/


/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

//--------------------Important values/Parameters--------------------------------//
//angle at which action is taken, in milliradians
const int ANGLE_TOLERANCE = 500;
//distance motor moves by default
const int MOTOR_STEP = 100;
//Height of the patient when on flat ground in mm
const int PATIENT_HEIGHT = 1200;
//Maximum height of legs in mm
const int MAX_LENGTH = 1825;

const int UP = 1;
const int DOWN = 0;


//Variables

//pitch is positive 'backwards' or going uphill (around Z axis). In milliradians
int pitch;
//roll is positive to the right (around X axis). In milliradians
int roll;

//Height of all four legs in mm
//ASSUMES DEVICE STARTS COMPLETELY LOWERED
int heightLeg1 = 0;
int heightLeg2 = 0;
int heightLeg3 = 0;
int heightLeg4 = 0;

//-----------------------------Functions----------------------------------------//

//Inputs: Leg identifier (shown above) as int, direction as  UP or DOWN, distance as int
//Outputs: none
void moveLeg(int identifier, bool direction, int amount){
    ;
}

//Inputs: none
//Outputs: none
//Handles logic of correcting for off-center gravity
void correctAngle(){
    if (pitch<0){
        if((legHeight3<=PATIENT_HEIGHT || legHeight4<=PATIENT_HEIGHT) && (legHeight1<MAX_HEIGHT && legHeight2<MAX_HEIGHT)){
            moveLeg(1,UP,MOTOR_STEP);
            moveLeg(2,UP,MOTOR_STEP);
        }
        else if(!(legHeight1<MAX_HEIGHT || legHeight2<MAX_HEIGHT) && (legHeight3>0 && legHeight4>0)){
            moveLeg(3,DOWN,MOTOR_STEP);
            moveLeg(4,DOWN,MOTOR_STEP);
        }
        else{
            //In this case, we can't do anything because presumably the legs are at max and min heights
        }
    }
}

//Inputs: X, Y, Z components of acceleration (assumed to be gravity) as doubles
//Outputs: None
//Calculates pitch and roll based on acceleration vector
//WRITES TO pitch, roll
void getDirection(double x, double y, double z, int identifier){
    pitch=1000*atan(x/y);
    roll=1000*atan(z/y);

    if(pitch<ANGLE_TOLERANCE){
        correctAngle();
    }
    else{
        ;//TODO Somehow check if, while level, it's at the right height. But we shouldn't have to??
    }
}

int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    while(1)
    {
        MAP_PCM_gotoLPM0();
    }
}