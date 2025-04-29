/*****************************************************************
 *Author: Patrick Laciuga
 * Co-Author:Ruth Massock
 * Fall 2024
 * Lab Section: 03
 * Created: 12/02/2024
 * Lab Number: Design Project
 * Description: Main program for joystick-controlled stepper motor.
 ******************************************************************/
#include "LcdDrivermsp430/Crystalfontz128x128_ST7735.h"
#include "LcdDrivermsp430/HAL_MSP_EXP430FR5994_Crystalfontz128x128_ST7735.h"
#include "grlib.h"
#include "driverlib.h"
#include <stdint.h>
#include <stdio.h>

// Global Variables

Graphics_Context g_sContext;
uint16_t JoyStickX, JoyStickY;
Timer_B_outputPWMParam MyTimerB;
Timer_A_initUpModeParam MyTimerA;

// Function Headers

void LCD_init(void);
void ADC_init(void);
void joyStick_init();
void configTimerA(uint16_t,uint16_t);
void myTimerADelay(uint16_t,uint16_t);
void configurePortIO(void);
void config_mkII_interrupts(void);
void myMotorDriver(void);
void myMotorController(void);
void handleJoystickInput(uint16_t,uint16_t);
void handleDebugMode(void);
void rotate30DegCW(void);
void newMotorController();
void patternExec(void);
void setupTimerB();
void startTimerB();
void stopTimerB();
typedef enum {OFFstby,CW,CCW, PATTERN} motorMode;
typedef enum {SLOW, NORMAL, FAST} motorSpeed;
motorSpeed speed;
motorMode motorTurn = OFFstby;
uint16_t delayMotor;
uint8_t PBS1,PBJ1;
uint8_t PBS2,PBS3;

#define YHIGH 0xA0B
#define YLOW 0x638
#define XHIGH 0xA0B
#define XLOW 0x638
#define XCENTER 0x763
#define YCENTER 0x763

uint16_t JoyStickX, JoyStickY;
// Global variable to track LED state
volatile int joystickState = 0;
volatile int joystickStateled=1;
volatile uint16_t ii=0;
enum {ON,OFF}joystickMode=OFF;
void toggle(void);


uint8_t MotorState = 0;


uint8_t motorSeq; // Global variable to hold the motor sequence step (0-7)

// Function prototypes

void myMotorDriver(void);
char buffer[100];
// Main function

void main(void) {
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    // Set up the GPIO for each LED based on motor sequence
        //Stop WDT
        WDT_A_hold(WDT_A_BASE);

        // Initialize Joystick
        joyStick_init();

        // Configure

        configurePortIO();

        // Activate Configuration
        PMM_unlockLPM5();

        // Initialize ADC
        ADC_init();

        // Initialize LCD
        LCD_init();

        setupTimerB();

        MotorState = 0;
        joystickMode=OFF;

        GPIO_clearInterrupt(GPIO_PORT_P4, GPIO_PIN3);

        // Configure Timer A for interrupts
        configTimerA(1000,TIMER_A_CLOCKSOURCE_DIVIDER_2);// Configure the timer parameters
        Timer_A_initUpMode(TIMER_A0_BASE,&MyTimerA);// Enable Local Interrupts
        Timer_A_enableInterrupt(TIMER_A0_BASE);
        __enable_interrupt();// Enable Global Interrupts
        while(1)
        {
            handleDebugMode();
            if(speed==FAST)
            {
                delayMotor=1562;
            } else
            {
                delayMotor=3125;
            }

            __disable_interrupt();
            Timer_A_disableInterrupt(TIMER_A0_BASE);
            configTimerA(delayMotor,TIMER_A_CLOCKSOURCE_DIVIDER_2);// Configure the timer parameters
            Timer_A_initUpMode(TIMER_A0_BASE,&MyTimerA);// Enable Local Interrupts
            Timer_A_enableInterrupt(TIMER_A0_BASE);
            __enable_interrupt();// Enable Global Interrupts
            Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
            __low_power_mode_0();
            __no_operation();
        }
}


//motor ISR
//FOR motorController
#pragma vector=TIMER0_A1_VECTOR

__interrupt void motorISR(void){
    newMotorController();
    // Clear Timer Interrupt Flag
    Timer_A_clearTimerInterrupt(TIMER_A0_BASE);
    Timer_A_stop(TIMER_A0_BASE);
    __low_power_mode_off_on_exit();
}


// LED ISR
//To Handle Toggle
#pragma vector = TIMER0_B0_VECTOR
__interrupt void TimerBISR(void){
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN1);
}


//Debug Function
// Function to handle debug mode

void handleDebugMode() {
    motorTurn = OFFstby;
    // Read push buttons states (e.g., from GPIO)
    PBS1 = GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN6);
    PBS2 = GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN5);
    PBS3= GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN2);//joybutton

    if ((!PBS1) && (!PBS2)) {
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN1);
        // Both buttons pressed, enter pattern mode
        joystickMode=OFF;

        MotorState = 0;
        motorTurn = PATTERN;


    } else if (!PBS1) {
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN1);

        joystickMode=OFF;
        motorTurn = CW;
        speed = FAST;




    } else if (!PBS2) {
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN1);


        joystickMode=OFF;
        motorTurn = CCW;
        speed = SLOW;


    } else
    {

        joystickMode=ON;

        ADC12_B_startConversion(ADC12_B_BASE, ADC12_B_START_AT_ADC12MEM0, ADC12_B_SEQOFCHANNELS);

        while (ADC12_B_getInterruptStatus(ADC12_B_BASE, 0, ADC12_B_IFG1) != ADC12_B_IFG1);

            JoyStickY = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_1);
            JoyStickX = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_0);

            ADC12_B_clearInterrupt(ADC12_B_BASE, 0, ADC12_B_IFG1);
                     if (JoyStickY > (YHIGH)){
                         GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
                         GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
                            joystickState = 1;
                            motorTurn = CW;
                            speed = FAST;
                        }
                     else if (JoyStickY < YLOW)
                    {
                        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
                        joystickState = 2; // Y-axis low
                        motorTurn = CW;
                        speed = SLOW;
                    }
                    else if (JoyStickX > (XHIGH ))
                    {
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
                        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN1);
                        joystickState = 3; // X-axis high
                        motorTurn = CCW;
                        speed = FAST;
                    }
                    else if (JoyStickX < XLOW)
                    {
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
                        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN1);
                        joystickState = 4; // X-axis low
                        motorTurn = CCW;
                        speed = SLOW;
                    }
                        else if (!PBS3)
                        {
                            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
                            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN1);
                            joystickState = 5;
                            motorTurn = PATTERN;
                        }
                    else {
                        joystickState = OFFstby;

                 }
    }


}
//ConfigIO Function
//Configure pins
void configurePortIO() {

        GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN7);
        GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN6);
        GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN5);
        GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN4);
        GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN1);
        GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);



        // Set input pins
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5,GPIO_PIN6);
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5,GPIO_PIN5);
        GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P6, GPIO_PIN2);  // jp2





        //Turn Off all Pins

        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
        GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN1);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);


}


// Function to drive the LEDs based on the motor sequence
void myMotorDriver()  {

    switch (MotorState) {
        case 0:
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); // A
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN4); //A_
            //  GPIO_setOutputHighOnPin(GPIO_PORT_J4, GPIO_PIN40); // mirror
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // B
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5); // B_
            break;

        case 1:
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); // A
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN4); //A_
          //  GPIO_setOutputHighOnPin(GPIO_PORT_J4, GPIO_PIN40); // mirror
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); // B
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5); // B_
            break;

        case 2:
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); // A
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN4); //A_
          //  GPIO_setOutputHighOnPin(GPIO_PORT_J4, GPIO_PIN40); // mirror
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); // B
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5); // B_
            break;

        case 3:
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); // A
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN4); //A_
          //  GPIO_setOutputHighOnPin(GPIO_PORT_J4, GPIO_PIN40); // mirror
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); // B
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5); // B_
            break;

        case 4:
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); // A
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN4); //A_
          //  GPIO_setOutputHighOnPin(GPIO_PORT_J4, GPIO_PIN40); // mirror
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // B
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5); // B_
            break;

        case 5:
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); // A
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN4); //A_
          //  GPIO_setOutputHighOnPin(GPIO_PORT_J4, GPIO_PIN40); // mirror
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // B
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5); // B_
            break;

        case 6:
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); // A
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN4); //A_
          //  GPIO_setOutputHighOnPin(GPIO_PORT_J4, GPIO_PIN40); // mirror
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // B
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5); // B_
            break;

        case 7:
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); // A
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN4); //A_
          //  GPIO_setOutputHighOnPin(GPIO_PORT_J4, GPIO_PIN40); // mirror
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // B
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5); // B_
            break;

        default:
            // A RED LP
            GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN7);
            // B RED EB
            GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN6);
            // ABAR BLUE EB
            GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5);
            // BBAR GREEN EB
            GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN4);
            break;
    }

}



void joyStick_init(){
    // JoyStick X
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN3, GPIO_TERNARY_MODULE_FUNCTION);

    // JoyStick Y
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2, GPIO_TERNARY_MODULE_FUNCTION);
}

// LCD_Init
// Configures mkII LCD display
// Inputs: none
// Returns: none

void LCD_init()
{
/* Initializes display */
Crystalfontz128x128_Init();
/* Set default screen orientation */
Crystalfontz128x128_SetOrientation(0);
/* Initializes graphics context */
Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128);
Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
Graphics_clearDisplay(&g_sContext);
}

// ADC_init
// Configures ADC to use joystick inputs
// Inputs: none
// Returns: none

void ADC_init(){
    //Initialize the ADC12B Module
    /*
    * Base address of ADC12B Module
    * Use internal ADC12B bit as sample/hold signal to start conversion
    * USE MODOSC 5MHZ Digital Oscillator as clock source
    * Use default clock divider/pre-divider of 1
    * Not use internal channel
    */
    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_ADC12OSC;
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap = ADC12_B_NOINTCH;
    ADC12_B_init(ADC12_B_BASE, &initParam);

    //Enable the ADC12B module
    ADC12_B_enable(ADC12_B_BASE);
    /*
    * Base address of ADC12B Module
    * For memory buffers 0-7 sample/hold for 64 clock cycles
    * For memory buffers 8-15 sample/hold for 4 clock cycles (default)
    * Enable Multiple Sampling
    */
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
      ADC12_B_CYCLEHOLD_16_CYCLES,
      ADC12_B_CYCLEHOLD_4_CYCLES,
      ADC12_B_MULTIPLESAMPLESENABLE);
    //Configure Memory Buffer
    /*
    * Base address of the ADC12B Module
    * Configure memory buffer 0
    * Map input A1 to memory buffer 0
    * Vref+ = AVcc
    * Vref- = AVss
    * Memory buffer 0 is not the end of a sequence
    */
    //  JoyStickXParam Structure

    ADC12_B_configureMemoryParam joyStickXParam = {0};
    joyStickXParam.memoryBufferControlIndex = ADC12_B_MEMORY_0;
    joyStickXParam.inputSourceSelect = ADC12_B_INPUT_A2;
    joyStickXParam.refVoltageSourceSelect = ADC12_B_VREFPOS_AVCC_VREFNEG_VSS;
    joyStickXParam.endOfSequence = ADC12_B_NOTENDOFSEQUENCE;
    joyStickXParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    joyStickXParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &joyStickXParam);

    //  JoyStickYParam Structure
    ADC12_B_configureMemoryParam joyStickYParam = {0};
    joyStickYParam.memoryBufferControlIndex = ADC12_B_MEMORY_1;
    joyStickYParam.inputSourceSelect = ADC12_B_INPUT_A15;
    joyStickYParam.refVoltageSourceSelect = ADC12_B_VREFPOS_AVCC_VREFNEG_VSS;
    joyStickYParam.endOfSequence = ADC12_B_ENDOFSEQUENCE;
    joyStickYParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    joyStickYParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &joyStickYParam);

    // Clear Interrupt
    ADC12_B_clearInterrupt(ADC12_B_BASE,0,ADC12_B_IFG1);


}

// configTimerA
// Configuration Parameters for TimerA
// Inputs: delayValue -- number of count cycles
//         clockDividerValue -- clock divider
// Returns: None

void configTimerA(uint16_t delayValue, uint16_t clockDividerValue)
{
    MyTimerA.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    MyTimerA.clockSourceDivider = clockDividerValue;
    MyTimerA.timerPeriod = delayValue;
    MyTimerA.timerClear = TIMER_A_DO_CLEAR;
    MyTimerA.startTimer = false;
}

void rotate30DegCW(void)
{
    int i=0;
    for(i=0; i<33; i++)
    {
        MotorState++;
        if(MotorState>7)
        {
            MotorState=0;
        }
        myMotorDriver();
         __delay_cycles(5000);
    }


}

void patternExec(void)
{
    MotorState = 0;
    myMotorDriver();
    //Using Pattern 3.
    //12 to 1
    rotate30DegCW();
     __delay_cycles(500000);
    //1 to 7
    int i=0;
    for(i=0;i<6;i++)
    {
        rotate30DegCW();
    }
     __delay_cycles(500000);
    //7 to 10
    i=0;
    for(i=0;i<3;i++)
    {
        rotate30DegCW();
    }
     __delay_cycles(500000);
    //10 to 4
    i=0;
    for(i=0;i<6;i++)
    {
        rotate30DegCW();
    }
     __delay_cycles(500000);
    //4 to 8
    i=0;
    for(i=0;i<4;i++)
    {
        rotate30DegCW();
    }
     __delay_cycles(500000);
    //8 to 2
    i=0;
    for(i=0;i<6;i++)
    {
        rotate30DegCW();
    }
     __delay_cycles(500000);
    //2 to 10
    i=0;
    for(i=0;i<8;i++)
    {
        rotate30DegCW();
    }
     __delay_cycles(500000);
    //10 to 7
    i=0;
    for(i=0;i<9;i++)
    {
        rotate30DegCW();
    }
     __delay_cycles(500000);
    //7 to 6
    i=0;
    for(i=0;i<11;i++)
    {
        rotate30DegCW();
    }
    __delay_cycles(500000);
    //6 to 12
    i=0;
    for(i=0;i<6;i++)
    {
        rotate30DegCW();
    }
    MotorState = 0;
    myMotorDriver();
}

void myTimerADelay(uint16_t delayValue, uint16_t clockDividerValue)
{

   configTimerA(delayValue,clockDividerValue);  // Configure the timer parameters
   Timer_A_initUpMode(TIMER_A0_BASE,&MyTimerA); // Initialize the timer
   Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);  // Start Timer
   while((TA0CTL&TAIFG) == 0);                   // Wait for TAIFG to become Set
   Timer_A_stop(TIMER_A0_BASE);                  // Stop timer
   Timer_A_clearTimerInterrupt(TIMER_A0_BASE);   // Reset TAIFG to Zero
}


void newMotorController(){
    switch(motorTurn){

    case OFFstby:
        startTimerB();
        break;
    case CW:
        stopTimerB();
        if(MotorState < 7) MotorState++;else MotorState = 0;

        break;
    case CCW:
        stopTimerB();
        if(MotorState >0) MotorState--;else MotorState = 7;


        break;
    case PATTERN:
        patternExec();
        break;

    default:
       MotorState = 0;
       break;
    }

    myMotorDriver();

    }


void setupTimerB(){
    TB0CCTL0 = CCIE;
    TB0CCR0 = 32767;
    TB0CTL = TBSSEL_1 | MC_1 | TBCLR;
}

void startTimerB(){
    TB0CTL |= MC_1;
}

void stopTimerB(){
    TB0CTL &= ~MC_3;
}
