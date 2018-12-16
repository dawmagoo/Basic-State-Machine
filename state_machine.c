/*
 *  Basic State Machine Example Using 
 *  Function Pointers and ISRs
 *  
 *  See "Microcontroller State Machine Implementation"
 *  <http://www.heptapod.com/lib/statemachines>
 * 
 *  Copyright 2015 Jakub Telatnik
 *  jakub [at] heptapod [dot] com
 *
 *  Permission is hereby granted, free of charge, to any person obtaining 
 *  a copy of this software and associated documentation files 
 *  (the "Software"), to deal in the Software without restriction,
 *  including without limitation the rights to use, copy, modify, merge,
 *  publish, distribute, sublicense, and/or sell copies of the Software,
 *  and to permit persons to whom the Software is furnished to do so,
 *  subject to the following conditions:
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
 *  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 *  NONINFRINGEMENT. 
 * 
 */
 
 /*
 * Adopted for PICSimLAB simulator (Board 1,PIC16F628A), 
 * <https://sourceforge.net/projects/picsim/> 
 * by Przemyslaw Slusarczyk, pslusarczyk [at] ujk [dot] edu [dot] pl 
 * Institute of Physics, Jan Kochanowski University, Kielce, Poland
 * 
 */

// CONFIG
#pragma config FOSC = EXTRCCLK  // Oscillator Selection bits 
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RA5/MCLR/VPP Pin Function Select bit (pin function is MCLR)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable bit
#pragma config CPD = OFF        // Data EE Memory Code Protection bit
#pragma config CP = OFF         // Flash Program Memory Code Protection bit

#include <xc.h>
#include <stdlib.h>

#define _XTAL_FREQ  1000000UL

// --------------------------------------------------------
//  ---------------  OUTPUTS DEFINITIONS ------------------    

//  Turn LED ON/OFF by using pin PB0
#define LED_ON()            PORTBbits.RB0 = 1;
#define LED_OFF()           PORTBbits.RB0 = 0;

// --------------------------------------------------------
// ----------------  INPUTS DEFINITIONS ------------------- 

#define PRESSED     0
#define RELEASED    1

int isRA1Pressed() {
    if(PORTAbits.RA1 == PRESSED) {          // if switch is pressed
        __delay_ms(50);                     // wait
        if(PORTAbits.RA1 == PRESSED) {      // if switch is still pressed
            __delay_ms(50);                 // wait
            if(PORTAbits.RA1 == RELEASED) { // if switch is released
                return 1;
            }   
        }
    }
    return 0;
}

// --------------------------------------------------------
// ----------------  TIMERS DEFINITIONS ------------------- 

//  Enable the blinking light by running Timer 0
#define LED_BLINK_START()   INTCONbits.TMR0IE = 1;  // enable Timer 0 interrupt
#define LED_BLINK_STOP()    INTCONbits.TMR0IE = 0;  // disable Timer 0 interrupt

//  Enable the timing out by running Timer 1
#define LED_TIMEOUT_TICKS   10 

volatile unsigned int led_timeout_counter;

void StopLEDTimeout(void){
    T1CONbits.TMR1ON = 0;   // disable Timer 1
}

void StartLEDTimeout(void){
    T1CONbits.TMR1ON = 0;   // disable Timer 1
    led_timeout_counter = LED_TIMEOUT_TICKS;
    T1CONbits.TMR1ON = 1;   // enable Timer 1
}


// --------------------------------------------------------
// -------- GENERIC STATE MACHINE DEFINITIONS -------------

#define QUEUE_SIZE     8

//  All the possible inputs of our state machine 
typedef enum{
    IN_NONE    = 0,     // No Event
    IN_BTN_PRESS,       // Button Press
    IN_TIMEOUT          // Timeout 
} TInputs;

// Each state will be implemented as a function 
// pStateHandler is a pointer to the functions.
typedef void ( *pStateHandler )(TInputs next_input);

// A structure that holds all the information needed to run the state machine
typedef struct StateMachine TStateMachine;

// Here is the state machine structure. 
struct StateMachine{
    pStateHandler           currentState;			// pointer to current state's function
    volatile TInputs        inQueue[QUEUE_SIZE];	// circular buffer of input inputs
    volatile unsigned char  queueCount;             // number of inputs events in queue
    unsigned char           queuePos;               // index of next input to be handled
};

// This function will add an input event to the queue
void AddInput(TStateMachine *sm, TInputs in_f){
    unsigned char i; 
    //make sure the input queue is not full
    if(sm->queueCount < QUEUE_SIZE){
        i = (unsigned char)(sm->queuePos + sm->queueCount); 
        i %= QUEUE_SIZE;
        sm->inQueue[i] = in_f;
        sm->queueCount++;
    }
    return;
}

// This function will return the next input from the queue
TInputs GetNextInput(TStateMachine *sm){
    TInputs next_input;
    if(sm->queueCount == 0){
        // queue is empty
        next_input = IN_NONE;
    } else{
        next_input = sm->inQueue[sm->queuePos];
        sm->queuePos++;
        if(sm->queuePos == QUEUE_SIZE)
            sm->queuePos = 0;
        sm->queueCount--;
    }
    return next_input;
}

// --- APPLICATION SPECIFIC STATE MACHINE DEFINITIONS -----

//  All states in the state machine, function prototypes
void LEDoff(TInputs next_input);
void LEDon(TInputs next_input);
void LEDblink(TInputs next_input);

// Definition of the input queue
volatile TInputs inQueue[QUEUE_SIZE];

//  Definition of the LED state machine 
TStateMachine st_machine =  {   
                            LEDoff,
                            (TInputs)inQueue,
                            0,
                            0
                        };

// Here are how the states will handle inputs. 
void LEDoff(TInputs next_input){
    switch(next_input){
        case IN_BTN_PRESS:      // LED Off + Button Press
            LED_ON();           
            StartLEDTimeout();  // start the timer
            st_machine.currentState = LEDon;  // State Transition
            break;
        default:
            break;
    }
}

void LEDon(TInputs next_input){
    switch(next_input){
        case IN_BTN_PRESS:      // LED On + Button Press
            StartLEDTimeout();  
            LED_BLINK_START();  
            st_machine.currentState = LEDblink; // State Transition 
            break;
        case IN_TIMEOUT:        // LED On + Timeout
            LED_OFF();    
            st_machine.currentState = LEDoff;   // State Transition
            break;
        default:
            break;
    }
}

void LEDblink(TInputs next_input){
    switch(next_input){
        case IN_BTN_PRESS:      // LED Blink + Button Press
        case IN_TIMEOUT:        // LED Blink + Timeout 
            LED_BLINK_STOP();   
            LED_OFF();          
            st_machine.currentState = LEDoff;   // State Transition 
            break;
        default:
            break;
    }
}
 

// --------------------------------------------------------
// ------------  INTERRUPT SERVICE ROUTINE ----------------

void interrupt isr(void){
    
    // Blinking RB0
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        PORTBbits.RB0 = (unsigned)~PORTBbits.RB0;
        INTCONbits.TMR0IF = 0;      // clear this interrupt condition
	 }

    // Timing out
	 if(PIE1bits.TMR1IE && PIR1bits.TMR1IF) {
        if(led_timeout_counter){ 
            led_timeout_counter--;  // decrement timeout counter
        }else{ 
            StopLEDTimeout();     // stop the timeout counter
            AddInput(&st_machine, IN_TIMEOUT); // timeout has occured 
        }
        PIR1bits.TMR1IF = 0;        // clear this interrupt condition
	 }
}


// --------------------------------------------------------
// ------------  INITIALIZATION FUNCTIONS ----------------- 

void initTimer0(void){
    OPTION_REGbits.T0CS = 0;  // internal clock
    OPTION_REGbits.T0SE = 1;  // increment on high-to-low transition on RA4 pin
    OPTION_REGbits.PSA = 0;   // prescaler is assigned to timer 0
    OPTION_REGbits.PS2 = 1;   // prescaler 1:256 - PS2:PS0 = 111
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;
  
    TMR0 = 0;                 // initial value
}

void initTimer1(void){
    T1CONbits.TMR1CS = 0;     // internal clock
    T1CONbits.T1OSCEN = 0;    // external oscillator is shut off
    T1CONbits.nT1SYNC = 1;    // do not synchronize external clock input 
    T1CONbits.T1CKPS1 = 1;    // prescaler 1:1 to 1:8 - T1CKPS1:T1CKPS0 = 00
    T1CONbits.T1CKPS0 = 0;
    
    TMR1H = TMR1L = 0;        // initial value
}

void systemInit(void){

    TRISA = 0xFF;         // port A as inputs
    TRISB = 0x00;         // port B as outputs 
    
    initTimer0();
    initTimer1();
       
    LED_OFF();
    
    PIE1bits.TMR1IE = 1;  // enable interrupts for timer 1 (timing out)   
    INTCONbits.PEIE = 1;  // enable peripheral interrupts
    ei();
}

int main(void){

    systemInit();                          
    
    while(1){        
        if(isRA1Pressed() == 1){
            AddInput(&st_machine, IN_BTN_PRESS);
        }
        
        if(st_machine.queueCount != 0){		// if queue not empty
            (st_machine.currentState)(GetNextInput(&st_machine)); 
        }
    }
}
