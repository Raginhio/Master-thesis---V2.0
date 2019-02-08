/*
 * Copyright 2017 David Ragnarsson
 * 
 * This software is free to use and modify for anyone.
 * No warranty is granted.
 */


/*
 * PWM signal on RC2 and GPIO on RA0 (LEDs on these pins for visualisation)(No leds on these pins on PCB)
 * HC-06 is used to send USART information to a bluetooth terminal 
 * Timer 4 will be used to have the timings when new ADC are to be made
 * 
 * OBS! all Timer init functions can be found in pmw_steering.c
 * All global variables are defined in main.h
 * 
 * all measurement interrupt functions can be found in ADC handler, including compare interrupt on timer3
 * 
 * At the top of ADChandler.c you can find the sampling order during active mode
 * 
 * Module go to sleep in timer4 interrupt in PWMsteering.c
 * It powers up in I2C interrupt
 * 
 * Power switch for measurement circuit is controlled by RC1
 */

#include "main.h"



void main(void) {
    
    
    //Start Switch transistor
    TRISCbits.RC1 = 0;      // Selected as output
    LATCbits.LATC1 = 0;     // set output low to start switch
    
    // Initializing all wanted functionality
    //init_default_pins();
    initClockFreq();        // function can be found in main.c
    init_peripherals();     // main.c
    init_pwm();             // function can be found in pwm_steering
    init_timer4();          // Timer4 will be used to start ADC meassurements
    init_timer3();          // Timer3 will be used to start ADC conversion on PWM amplitude
    init_ADC();
    init_FVR();             // Can be found in ADChandler
    init_interrupt();       // function can be found in interrupt_handler
    //init_eusart();          // function can be found in EUSART_control. Check peripherals
    init_I2C();             // Initializing I2C master in I2C_handler.c
    init_I2C_Slave();
    I2C_Master_DAC((int) 2048);
    
    //int a=0,c=0,d=1;
    TRISAbits.RA0 = 0;      // Sets RA0 to output
    PORTAbits.RA0 = 0;      // Sets output low
    
    //CCPR1L= 0x3e;
    
    // Main loop
    while(1){
        ClrWdt();   // Watchdog timer ensures the software never get stucked
        
        if(VALID_DATA && (I2C_REQUEST == 0)){
            measure_sensor();       //ADChandler.c
        }
        
        if(ADC_CALCULATION == 1){
            ADC_interrupt();
            ADC_CALCULATION = 0;
        }
        
        if(TIMER4_UPDATE == 1){
            Timer4_interrupt();
            TIMER4_UPDATE = 0;
        }
        
    }
    return;
}



void initClockFreq(void){
    OSCCONbits.OSTS  = 1;          // Device is running from the clock defined by FOSC
    OSCCONbits.SCS1  = 1;          // Internal oscillator block 
    OSCCONbits.IRCF  = 0b111;      // Select frequency 0b111 = 16MHz
                                   // 0b100 = 2 MHz
                                   // 0b011 = 1 MHz
}

void init_peripherals(void){
    PMD0bits.TMR1MD = 1;     // Module disable
    PMD0bits.TMR5MD = 1;
    PMD0bits.TMR6MD = 1;
    PMD0bits.UART1MD = 1;
    PMD0bits.UART2MD = 1;
    
    PMD1bits.CCP2MD = 1;     
    PMD1bits.CCP3MD = 1;
    PMD1bits.CCP4MD = 1;
    
    PMD2bits.CMP1MD = 1;
    PMD2bits.CMP2MD = 1;
    PMD2bits.CTMUMD = 1;
}


void init_default_pins(void){
    TRISA = 0x00;
    TRISB = 0x00;
    TRISC = 0x00;
    
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;
}

