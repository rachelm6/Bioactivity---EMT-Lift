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
#include <ti/devices/msp432p4xx/inc/msp.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

//--------------------Important values/Parameters--------------------------------//
//The permissible deviation of the angle between the acceleration vector and the y axis in milliradians
const int ANGLE_TOLERANCE = 50;
//Leg height of when device is on flat ground in mm
const int PATIENT_HEIGHT = 1200;
//Maximum height of legs in mm
const int MAX_HEIGHT = 1825;
//Height which device increases or decreases on button press in mm
const int STEP_HEIGHT = 50;
//Logical OR of pin numbers for direction output pins for 1, 2, 3, 4 IN ORDER
const uint_fast8_t DIRECTION_PINS = GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7;
//Encoder Pins with interrupts
const uint_fast8_t ENCODER_PINS_I = GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN4 | GPIO_PIN6;
//Encoder Pins without interrupts
const uint_fast8_t ENCODER_PINS = GPIO_PIN1 | GPIO_PIN3 | GPIO_PIN5 | GPIO_PIN7;
//Boolean value of output pin which causes legHeight to increase
const bool UP = 1;
//Boolean value of output pin which causes legHeight to decrease
const bool DOWN = 0;
//Encoder ticks per revolution
const int ENCODER_VALUE = 24;
//Height per revolution of WORM GEAR in mm
const float PITCH = 12.7;
//Rate in Hz that control function is called on stored 'desired heights'
const int CONTROLLER_RATE = 100;

const float pi = 3.14159;


//Map CCRs 1,2,3,4 to pins 4.0,4.1,4.2,4.3 respectively
const uint8_t port4_mapping[] =
{//Port 4
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
int legHeight[4]={0,0,0,0};

//Desired height of all four legs in mm
//ASSUMES DEVICE STARTS COMPLETELY LOWERED
int desHeight[4]={0,0,0,0};

//speed of all four legs
float speed[4]={0,0,0,0};

//times used for speed calculations
//row 0 accessed by encoders, row 1 accessed by controller
int lastTime[2][4]={{0,0,0,0},{0,0,0,0}};

//-----------------------------Functions----------------------------------------//

//Inputs: timer as standard timer label, interrupt number, value for timer to count down from as uint32_t
//Outputs: None
//Configures Timer32 in periodic mode without starting it
void configTimer32Periodic(uint32_t timer, uint32_t interruptNumber, uint32_t initialValue){
    Timer32_initModule(timer, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_setCount(timer, initialValue);
    if (interruptNumber == TIMER32_0_INTERRUPT){
        //Timer32_registerInterrupt(interruptNumber, *T32_INT0_IRQHandler);
        Interrupt_enableInterrupt(TIMER32_0_INTERRUPT);
    }
    if (interruptNumber == TIMER32_1_INTERRUPT){
        Timer32_clearInterruptFlag(TIMER32_1_BASE);
        //Timer32_registerInterrupt(interruptNumber, *T32_INT1_IRQHandler);
        //Interrupt_enableInterrupt(TIMER32_1_INTERRUPT);
    }

}

//Inputs: port as standard GPIO port labels, pin (or logical or of multiple pins) as standard GPIO labels
//Outputs: None
//Configures port.pins as an output and sets to low
void configOutput(uint_fast8_t port, uint_fast8_t pin){
    GPIO_setAsOutputPin(port, pin);
    GPIO_setOutputLowOnPin(port, pin);
}

//Inputs: port as standard GPIO port labels, pin (or logical or of multiple pins) as standard GPIO labels
//Outputs: None
//Configures port.pins as an input WITh PULLUP RESISTOR then enables and clears interrupt
void configButton(uint_fast8_t selectedPort, uint_fast8_t selectedPins, uint32_t interruptNumber){
    MAP_GPIO_setAsInputPinWithPullUpResistor(selectedPort, selectedPins);
    MAP_GPIO_clearInterruptFlag(selectedPort, selectedPins);
    MAP_GPIO_enableInterrupt(selectedPort, selectedPins);
    MAP_Interrupt_enableInterrupt(interruptNumber);
}

//Inputs: port as standard GPIO port labels, pin (or logical or of multiple pins) as standard GPIO labels
//Outputs: None
//Configures port.pins as an input then enables and clears interrupt
void configInputWithInterrupt(uint_fast8_t selectedPort, uint_fast8_t selectedPins, uint32_t interruptNumber){
    MAP_GPIO_setAsInputPin(selectedPort, selectedPins);
    MAP_GPIO_clearInterruptFlag(selectedPort, selectedPins);
    MAP_GPIO_enableInterrupt(selectedPort, selectedPins);
    MAP_Interrupt_enableInterrupt(interruptNumber);
}


//Inputs: port as standard GPIO port labels, pin (or logical or of multiple pins) as standard GPIO labels
//Outputs: None
//Configures port.pins as an input and clears interrupt
void configInput(uint_fast8_t selectedPort, uint_fast8_t selectedPins){
    MAP_GPIO_setAsInputPin(selectedPort, selectedPins);
    MAP_GPIO_clearInterruptFlag(selectedPort, selectedPins);
}

//Inputs: the period (in clock ticks) for timer A0 as an int, the value for the ccr to count up to as an int.
//ccrValue must be less than period
//Outputs: none
// Configures Timer_A0 for PWM generation on CCR 1-4
// Timer_A0's Period is period, duty cycle of given CCR is TIMER_A0->CCR[n]/period.
void configTimerA0forPWM(int period, int ccrValue){
    //    TIMER_A0->CTL = 0x0110;
        TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK // Sources Timer_A0 from the A Clock ...
                            | TIMER_A_CTL_ID__1 // ... with a divider of 1 ...
                            | TIMER_A_CTL_MC__UP; // accumulating in Up mode (ccr0 stores period)

        TIMER_A0->CCR[0] = period; // Sets period

        //Set CCRs for Compare mode with Toggle/Set output mode
        TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_6;
        TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_6;
        TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_6;
        TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_6;

        // Initialize CCRs so it starts in reliable state (50% duty cycle chosen for testing's sake)
        TIMER_A0->CCR[1]  = ccrValue;
        TIMER_A0->CCR[2]  = ccrValue;
        TIMER_A0->CCR[3]  = ccrValue;
        TIMER_A0->CCR[4]  = ccrValue;
}


//Inputs: Leg identifier (shown above) as int, direction as  UP or DOWN, dutyCycle as int between 0-TimerA period
//Outputs: none
//Handles hardware-level articulation (ie writing to pins and registers)
void moveLeg(int identifier, bool direction, int dutyCycle){
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
    TIMER_A0->CCR[identifier]  = dutyCycle;

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

//Inputs: Leg identifier (shown above) as int, height as int from 0 to MAX_HEIGHT
//Outputs: none
//Laplace-derived control function for generating PWM based on height difference
void lowLevelController(int identifier,  int height){
    int dutyCycle=0;
    int direction;
    //TODO time-domain controller (PID?)

    //Get appropriate direction value
    if(dutyCycle<0){
        dutyCycle=abs(dutyCycle);
        direction=DOWN;
    }
    else{
        direction=UP;
    }

    moveLeg(identifier,direction,dutyCycle);
}

//Inputs: none
//Outputs: none
//Handles logic of correcting for off-center gravity
void correctAngle(){
    if (pitch<0){
        if((legHeight[2]<=PATIENT_HEIGHT || legHeight[3]<=PATIENT_HEIGHT) && (legHeight[0]<MAX_HEIGHT && legHeight[1]<MAX_HEIGHT)){
            //moveLeg(1,UP,MOTOR_STEP);
            //moveLeg(2,UP,MOTOR_STEP);
        }
        else if(!(legHeight[0]<MAX_HEIGHT || legHeight[1]<MAX_HEIGHT) && (legHeight[2]>0 && legHeight[3]>0)){
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

    CS_setDCOFrequency(24000000); //24MHz DCO & CPU
    CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ); //128kHz for REF0
    CS_setExternalClockSourceFrequency(20000,24000000);   //HFXT to 20MHz, LFXT to 20kHz
    CS_startHFXT(false);  //start HFXT
    CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1); //20Mhz SM Clock from HFXT
    //CS_initClockSignal(CS_ACLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1); // 20MHz for AClk, from SMCLK
    configTimerA0forPWM(1000,500); //set up PWM with period of 1000 and at 50% duty cycle

    configTimer32Periodic(TIMER32_0_BASE, TIMER32_0_INTERRUPT, SystemCoreClock/CONTROLLER_RATE);//Set up Timer32 to run at designated frequency
    //Timer32_startTimer(TIMER32_0_BASE,0);    //Start T32 0 which calls the controller

    configOutput(GPIO_PORT_P4, DIRECTION_PINS); //configure pins as output for direction

    //copy output from Timer A0 CCRs to pins 4.0-4.3 (PWM)
    PMAP_configurePorts(port4_mapping, PMAP_P4MAP, 4, PMAP_ENABLE_RECONFIGURATION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    //Configure remaining GPIO (buttons and encoders)
    configButton(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4, INT_PORT1);
    configInput(GPIO_PORT_P2, ENCODER_PINS);
    configInputWithInterrupt(GPIO_PORT_P2, ENCODER_PINS_I, INT_PORT2);

    int ccrValue=0;
    int i;
    while(1)
    {
        //MAP_PCM_gotoLPM0();

        for(i=0;i<10000000;i++){
            if(i % 10000==0){
                ccrValue++;
                TIMER_A0->CCR[1]  = ccrValue;
                TIMER_A0->CCR[2]  = ccrValue;
                TIMER_A0->CCR[3]  = ccrValue;
                TIMER_A0->CCR[4]  = ccrValue;
                debug=ccrValue;
                //debug = TIMER_A0->CCTL[1];
            }
        }
        for(i=0;i<10000000;i++){
            if(i % 10000==0){
                ccrValue--;
                TIMER_A0->CCR[1]  = ccrValue;
                TIMER_A0->CCR[2]  = ccrValue;
                TIMER_A0->CCR[3]  = ccrValue;
                TIMER_A0->CCR[4]  = ccrValue;
                debug=ccrValue;
            }
        }
    }
}

//Interrupt handler for built-in button presses
//Inputs: none
//Outputs: none
void PORT1_IRQHandler(void){
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1,status);

    if (GPIO_PIN1 & status){
        // handle left button press event
        desHeight[0]+=STEP_HEIGHT;
        desHeight[1]+=STEP_HEIGHT;
        desHeight[2]+=STEP_HEIGHT;
        desHeight[3]+=STEP_HEIGHT;

    }

    else if(GPIO_PIN4 & status){
        // handle right button press event
        desHeight[0]-=STEP_HEIGHT;
        desHeight[1]-=STEP_HEIGHT;
        desHeight[2]-=STEP_HEIGHT;
        desHeight[3]-=STEP_HEIGHT;
    }
}

//Interrupt handler for Encoders
void PORT2_IRQHandler(){
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, status);
    //useful value to have
    float DELTA_H = PITCH / ENCODER_VALUE;

    if (GPIO_PIN0 & status){
        // leg 1 encoder interrupt
        if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN1)){
            legHeight[0]+=DELTA_H;
        }
        else{
            legHeight[0]-=DELTA_H;
        }

        speed[0]=DELTA_H/(0xffffffff-(Timer32_getValue(TIMER32_1_BASE))/SystemCoreClock);
        Timer32_setCount(TIMER32_1_BASE,0xffffffff);
    }

    if (GPIO_PIN2 & status){
        // leg 2 encoder interrupt
        if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN3)){
            legHeight[1]+=DELTA_H;
        }
        else{
            legHeight[1]-=DELTA_H;
        }
        speed[1]=DELTA_H/Timer32_getValue(TIMER32_0_BASE);
    }

    if (GPIO_PIN4 & status){
        // leg 3 encoder interrupt
        if(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN1)){
            legHeight[2]+=DELTA_H;
        }
        else{
            legHeight[2]-=DELTA_H;
        }

        speed[2]=DELTA_H/(0xffffffff-(Timer32_getValue(TIMER32_1_BASE))/SystemCoreClock);
        Timer32_setCount(TIMER32_1_BASE,0xffffffff);
    }

    if (GPIO_PIN6 & status){
        // leg 4 encoder interrupt
        if(GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN3)){
            legHeight[3]+=DELTA_H;
        }
        else{
            legHeight[3]-=DELTA_H;
        }
        speed[3]=DELTA_H/Timer32_getValue(TIMER32_0_BASE);
    }
}

//Interrupt handler for Timer32
void T32_INT0_IRQHandler(void)
{
    Timer32_clearInterruptFlag(TIMER32_0_BASE);

    lowLevelController(1,desHeight[0]);
    lowLevelController(2,desHeight[1]);
    lowLevelController(3,desHeight[2]);
    lowLevelController(4,desHeight[3]);
}
