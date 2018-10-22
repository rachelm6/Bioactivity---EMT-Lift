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

DEFAULT WIRING:
_____Leg 1___2____3____4
CCR    1     2    3    4
PWM   4.0   4.1  4.2  4.3
Dir   4.4   4.5  4.6  4.7
*/


/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

//--------------------Important values/Parameters--------------------------------//
//The permissible deviation of the angle between the acceleration vector and the y axis in milliradians
const int ANGLE_TOLERANCE = 50;
//distance motor moves by default
const int MOTOR_STEP = 100;
//Leg height of when device is on flat ground in mm
const int PATIENT_HEIGHT = 1200;
//Maximum height of legs in mm
const int MAX_HEIGHT = 1825;
//Logical OR of pin numbers for direction output pins for 1, 2, 3, 4 IN ORDER
const uint_fast8_t DIRECTION_PINS = GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7;
//Boolean value of output pin which causes legHeight to increase
const bool UP = 1;
//Boolean value of output pin which causes legHeight to decrease
const bool DOWN = 0;
const float pi = 3.14159;

//Map CCRs 1,2,3,4 to pins 4.0,4.1,4.2,4.3 respectively
const uint8_t port4_mapping[] =
{
        PMAP_TA0CCR1A,  PMAP_TA0CCR2A,  PMAP_TA0CCR3A, PMAP_TA0CCR4A,
        PMAP_NONE,      PMAP_NONE,      PMAP_NONE,   PMAP_NONE
};

//Variables

int debug;

//Pitch is rotation around the Z axis, positive 'backwards' or going uphill following right-hand-rule. In milliradians.
int pitch;
//roll is positive to the right (around X axis). In milliradians
int roll;

//Height of all four legs in mm
//ASSUMES DEVICE STARTS COMPLETELY LOWERED
int legHeight1 = 0;
int legHeight2 = 0;
int legHeight3 = 0;
int legHeight4 = 0;

//-----------------------------Functions----------------------------------------//

//Inputs: port as standard GPIO port labels, pin (or logical or of multiple pins) as standard GPIO labels

void configOutput(uint_fast8_t port, uint_fast8_t pin){
    GPIO_setAsOutputPin(port, pin);
    GPIO_setOutputLowOnPin(port, pin);
}

//Inputs: the period (in clock ticks) for timer A0 as an int, the value for the ccr to count up to as an int.
//ccrValue must be less than period
//Outputs: none
// Configures Timer_A0 for PWM generation on CCR 1-4
// Timer_A0's Period is period, duty cycle of given CCR is TIMER_A0->CCR[n]/period.
void configTimerA0forPWM(int period, int ccrValue){
    //    TIMER_A0->CTL = 0x0110;
        TIMER_A0->CTL = TIMER_A_CTL_SSEL__ACLK // Sources Timer_A0 from the A Clock ...
                            | TIMER_A_CTL_ID__1 // ... with a divider of 1 ...
                            | TIMER_A_CTL_MC__UP; // accumulating in Up mode (ccr0 stores period)
        TIMER_A0->CCR[0] = period; // Sets period

        //Set CCRs for Compare mode with Toggle/Set output mode
        TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_6;

        // Initialize CCRs so it starts in reliable state (50% duty cycle chosen for testing's sake)
        TIMER_A0->CCR[1]  = ccrValue;
        TIMER_A0->CCR[2]  = ccrValue;
        TIMER_A0->CCR[3]  = ccrValue;
        TIMER_A0->CCR[4]  = ccrValue;
}


//Inputs: Leg identifier (shown above) as int, direction as  UP or DOWN, amount as int between 0-1000 for duty cycle
//Outputs: none
void moveLeg(int identifier, bool direction, int amount){
    int directionPin;
    //pick correct pin for setting direction
    switch(identifier){
        case 1:
            directionPin=GPIO_PIN4;
            break;
        case 2:
            directionPin=GPIO_PIN5;
            break;
        case 3:
            directionPin=GPIO_PIN6;
            break;
        case 4:
            directionPin=GPIO_PIN7;
            break;
        default:
            break;
    }

    //set CCR period to appropriate duty cycle
    TIMER_A0->CCR[identifier]  = amount;

    //set direction pin high or low
    switch(direction){
        case 0:
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, directionPin);
            break;
        case 1:
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, directionPin);
            break;
        default:
            break;
    }
}

//Inputs: Leg identifier (shown above) as int, direction as  UP or DOWN, height as int from 0 to MAX_HEIGHT
//Outputs: none
//Laplace-derived control function for generating PWM based on height difference
void lowLevelController(int identifier, bool direction, int height){
    int dutyCycle;
    //TODO time-domain controller (PID?)

    //Corrects values for a negative duty cycle (ie takes magnitude and flips direction)
    if(dutyCycle<0){
        dutyCycle=abs(dutyCycle);
        direction=!direction;
    }
    moveLeg(identifier,direction,dutyCycle);
}

//Inputs: none
//Outputs: none
//Handles logic of correcting for off-center gravity
void correctAngle(){
    if (pitch<0){
        if((legHeight3<=PATIENT_HEIGHT || legHeight4<=PATIENT_HEIGHT) && (legHeight1<MAX_HEIGHT && legHeight2<MAX_HEIGHT)){
            //moveLeg(1,UP,MOTOR_STEP);
            //moveLeg(2,UP,MOTOR_STEP);
        }
        else if(!(legHeight1<MAX_HEIGHT || legHeight2<MAX_HEIGHT) && (legHeight3>0 && legHeight4>0)){
            //moveLeg(3,DOWN,MOTOR_STEP);
            //moveLeg(4,DOWN,MOTOR_STEP);
        }
        else{
            //In this case, we can't do anything because presumably the legs are at max and min heights
        }
    }
    else if(pitch>0){

    }
}

//Inputs: X, Y, Z components of acceleration (assumed to be gravity) as doubles
//Outputs: None
//Calculates pitch and roll based on acceleration vector
//WRITES TO pitch, roll
void getDirection(double x, double y, double z, int identifier){
    pitch=1000*atan(x/y);
    roll=1000*atan(z/y);

    if(abs(pitch-pi)>ANGLE_TOLERANCE){
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

    CS_setDCOFrequency(12000000); //12MHz DCO & CPU
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); //12Mhz SM Clock from DCO
    CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ); //128kHz for REF0
    CS_setExternalClockSourceFrequency(20000,10000000);   //HFXT to 20MHz, LFXT to 20kHz
    CS_startHFXT(0);  //start HFXT
    CS_initClockSignal(CS_ACLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1); // 20MHz for AClk, from HFXT
    configTimerA0forPWM(1000,1000); //set up PWM

    configOutput(GPIO_PORT_P4, DIRECTION_PINS); //configure pins as output for direction

    //copy output from Timer A0 CCRs to pins 4.0-4.3 (PWM)

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4,   GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1, 0x00);

    PMAP_configurePorts(port4_mapping, PMAP_P4MAP, 4, PMAP_ENABLE_RECONFIGURATION);

    int ccrValue=0;
    int i;
    while(1)
    {
        //MAP_PCM_gotoLPM0();

        for(i=0;i<10000000;i++){
            if(i % 10000==0){
                //ccrValue++;
                TIMER_A0->CCR[1]  = ccrValue;
                TIMER_A0->CCR[2]  = ccrValue;
                TIMER_A0->CCR[3]  = ccrValue;
                TIMER_A0->CCR[4]  = ccrValue;
                //debug=ccrValue;
                debug = TIMER_A0->CCTL[1];
            }
        }
        for(i=0;i<10000000;i++){
            if(i % 10000==0){
                //ccrValue--;
                TIMER_A0->CCR[1]  = ccrValue;
                TIMER_A0->CCR[2]  = ccrValue;
                TIMER_A0->CCR[3]  = ccrValue;
                TIMER_A0->CCR[4]  = ccrValue;
                //debug=ccrValue;
            }
        }
    }
}
