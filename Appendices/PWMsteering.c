/*
 * File:   pwm_steering.c
 * Author: davrag
 *
 * Created on den 2 februari 2017, 14:23
 */

/*
 * Timer3 interrupt can be found in ADChandler.c
 */

#include "PWMsteering.h"

void init_pwm(void) {
    // PWM setup. This setting will give a PWM frequency of 2kHz
            // PWM setup. This setting will give a PWM frequency of 2kHz
    TRISCbits.RC2 = 1;          // Disable the PWM pin. RC2/P1A
    ANSELCbits.ANSC2 = 0;
    
    PSTR1CONbits.STR1A = 1;     // Sets PWM signal on P1A
    CCPTMRS0bits.C1TSEL = 0b00; // PWM modes use Timer2
    ECCP1ASbits.CCP1AS = 0b00;  // Auto-shutdown is disable
    CCP1CONbits.CCP1M = 0b1100; // PWM mode; P1A, P1B, P1C, P1D active-high
    CCP1CONbits.P1M = 0b00;     // Single output; P1A, P1B, P1C, P1D controlled by steering
    PSTRCONbits.STRA = 1;       // P1A(RC2) has PWM waveform with polarity control from CCPxM
    
    if(HF_CLOCK == 16000000){
        PR2 = 0x7c;                 //HF_CLOCK/2000/4/16 - 1; // 0x7c;// PWM period   (This will also be the value which cause an interrupt on timer2)
        CCPR1L= 0x3e;               // 10-bit duty cycle
        CCP1CONbits.DC1B = 0b00;    // LSb 2-bit 
        T2CONbits.T2CKPS = 0b10;    // prescaler = 16
    }    
    else if(HF_CLOCK == 2000000){
        PR2 = 0xf8;                 //HF_CLOCK/2000/4/16 - 1; // 0x7c;// PWM period   (This will also be the value which cause an interrupt on timer2)
        CCPR1L= 0x7c;               // 10-bit duty cycle
        CCP1CONbits.DC1B = 0b00;    // LSb 2-bit 
        T2CONbits.T2CKPS = 0b00;    // prescaler = 1
    }
    else if(HF_CLOCK == 1000000){
        PR2 = 0x7c;                 //HF_CLOCK/2000/4/16 - 1; // 0x7c;// PWM period   (This will also be the value which cause an interrupt on timer2)
        CCPR1L= 0x3e;               // 10-bit duty cycle
        CCP1CONbits.DC1B = 0b00;    // LSb 2-bit 
        T2CONbits.T2CKPS = 0b00;    // prescaler = 1
    }
    
    
    // Configure Timer2 for PWM
    PIR1bits.TMR2IF = 0;        // Clears timer2 interupt flag
    T2CONbits.TMR2ON = 1;       // Starts Timer2
    
    PIR1bits.TMR2IF = 0;
    while(PIR1bits.TMR2IF ==0){ // Waits for Timer2 overflows
        ;
    }
    //TRISCbits.RC2 = 0;          // Enable output driver/P1A
    PWM1CONbits.P1RSEN = 1;
    ECCP1ASbits.CCP1ASE = 0;
}

void init_timer4(void){
    
    // Configure Timer4 for ADC measurements
    PIR5bits.TMR4IF = 0;        // Clears timer4 interupt flag
    
    //if(HF_CLOCK == 16000000){
        T4CONbits.T4CKPS = 0b11;    // prescaler = 16
        T4CONbits.T4OUTPS = 0b1111; // postscaler = 16
        PR4 = 0xff;
        
        //PR4 = 0x7c;//HF_CLOCK/2000/4/4 - 1; //0x7c;// PWM period (will give interrupt 4 times faster than timer2)
    //}
   /* else if(HF_CLOCK == 2000000){
        T4CONbits.T4CKPS = 0b00;    // prescaler = 1
        PR4 = 0x3e;//HF_CLOCK/8000/4/1 - 1; //0x7c;// PWM period (will give interrupt 4 times faster than timer2)
    }
    else if(HF_CLOCK == 1000000){
        T4CONbits.T4CKPS = 0b00;    // prescaler = 1
        PR4 = 0x1e;//HF_CLOCK/8000/4/1 - 1; //0x7c;// PWM period (will give interrupt 4 times faster than timer2)
    } */   
        
    PIE5bits.TMR4IE = 1;    // enable interrupt
    T4CONbits.TMR4ON = 1;       // Starts Timer4

}

// Timer 1 will use compare mode to set when an ADC conversion should start
// Current configuration are configured for 16 MHz operation
void init_timer3(void){
    T3CONbits.TMR3CS = 0b00;    // FOSC/4
    T3CONbits.T3CKPS = 0b01;    // prescaler 2
    T3CONbits.T3RD16 = 1;       // 16-bit operation
    //CCPTMRS0bits.C3TSEL = 0b01; // CCP3 uses timer 3 for compare
    CCPTMRS1bits.C5TSEL = 0b01;
    
    T3CONbits.TMR3ON = 0;   // disable timer3
    
    TMR3L = 0;      // Be sure timer starts at 0
    TMR3H = 0;
    
    CCPR3H = 0x01;  // 0x2ee = 750 which will be 3/4 of the PWMs duty cycle
    CCPR3L = 0x77;
 
    CCPR5H = 0x01;  // 875 in hex
    CCPR5L = 0x77;
    //CCPR5H = 0x03;  // 875 in hex
    //CCPR5L = 0x6B;
    
    CCP5CONbits.CCP5M = 0b1011;     // Generate software interrupt + starts AD conversion
    //CCP3CONbits.CCP3M = 0b1010;     // Generate software interrupt on match (No timer reset)
    
    //PIE4bits.CCP3IE = 0;
    PIE4bits.CCP5IE = 0;
    
}


void Timer2_interrupt(void){
        static int i=0;
        i++;
        PIR1bits.TMR2IF = 0;     // Clearing interrupt flag for timer2
            
            if(i >= 3000){
                LATAbits.LA0 ^= 1;   // toggle led RA0
                i = 0;
            } 
}


void Timer4_interrupt(void){
        PIR5bits.TMR4IF = 0;
        I2C_COUNTER++;
        uint16_t limit;
        
        if(TEMPERATURE_REQUEST && I2C_REQUEST){      // Check if it's time to measure temperature
            I2C_REQUEST = 0;                // Set again in end of temperature calculation
            ADCON0bits.CHS = 0b01011;      // AN11 (RB4) temperature control cell

            CCPR5H = 0x01;          // 0x2ee = 750 which will be 3/4 of the PWMs duty cycle
            CCPR5L = 0x77;          // 0x177 = 375    
            while(ADCON0bits.GODONE){ // Wait for last ADC to finish
                ;
            }
            PIR4bits.CCP5IF = 0;    // Clear interrupt flag
            PIE4bits.CCP5IE = 1;    // Enable compare mode for timer 3
            while(TMR2 != 0);       // waits for reset on timer2
            T3CONbits.TMR3ON = 1;   // enable timer 3       // Temperature is measured every 5 request from electrotech
        }
        else if(!TEMPERATURE_REQUEST){
            while(ADCON0bits.GODONE){
                ;
            }
        
        ADCON0bits.CHS = 0b01101;   // Nerst voltage channel
        ADCON0bits.GODONE = 1;      // Restart conversion for nerst cell
        }
        
        if(HF_CLOCK == 16000000){
            limit = 180;
        }
        else if(HF_CLOCK == 2000000){
            limit = 22;
        }
        
        if(I2C_COUNTER >= limit){     // 180 = 3 sec in 16MHz  (22 = 3 sec in 2MHz) 
            Go2Sleep();
        }
        //ADCON0bits.GODONE = 1;    // Stars a new ADC conversion
}

void Go2Sleep(void){
    I2C_COUNTER = 0;
    LATCbits.LATC1 = 1;             // power down circuit
    VALID_DATA = 0;
    
    PIE1bits.ADIE = 0;              // disable ADC interrupt
    PIE5bits.TMR4IE = 0;            // disable timer4 interrupt
    PIE4bits.CCP5IE = 0;            // disable compare interrupt
    //PIE4bits.CCP3IE = 0;            // disable compare interrupt
    
    SLEEP();
    ClrWdt();
}
