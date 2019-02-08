/*
 * File:   pwm_steering.c
 * Author: davrag
 *
 * Created on den 2 februari 2017, 14:23
 */

#include "INTERRUPThandler.h"


// Initializing interrupt configuration
void init_interrupt(){
        /*
         * Configuration for timer interrupt on timer2
         * Configuration for ADC on AN11 RB4         
        */
    // ADC interrupt
    PIR1bits.ADIF = 0;      // ADC interrupt flag is cleared
    PIE1bits.ADIE = 1;      // Enable ADC interrupt
    
    /*// No priority interrupts
    RCONbits.IPEN   = 0;    // Disable priority levels on interrupts 
    INTCONbits.GIE  = 1;    // Enables all unmasked interrupts  (This instruction is dependent on IPEN bit)*/
    
    // Set all interrupts to low priority
    IPR1 = 0;
    IPR2 = 0;
    IPR3 = 0;
    IPR4 = 0;
    IPR5 = 0;
    
    IPR3bits.SSP2IP = 1;    // Sets I2C slave interrupt to high priority
    
    // High and low priority interrupts
    RCONbits.IPEN = 1;      // Enable priority levels on interrupts
    INTCONbits.GIEH = 1;    // Enable high priority interrupt
    INTCONbits.GIEL = 1;    // Enable low priority interrupt
    
    INTCONbits.PEIE = 1;    // Enables all unmasked peripheral interrupts (This instruction is dependent on IPEN bit)
    PIE1bits.TMR2IE = 0;    // Disables interrupt on timer2 to match PR2
    PIE5bits.TMR4IE = 1;    // Enables interrupt on timer4 to match PR4
}
// Setup for interrupt function. Timer2 interrupt will be first utilized
void low_priority interrupt ISR(){
    
    static uint8_t n = 0;
    //PIR2bits.BCLIF = 0;
    // EUSART read interrupt
    /*if(PIR4bits.CCP3IF == 1){
        Compare_interrupt();
    }*/
    if(PIR4bits.CCP5IF == 1){
        //Compare5_interrupt();
        
            // Change compare value to a half PWM period
            if(CCPR5L == 0x77){
                CCPR5H = 0x01;  // 0x1f4 = 500
                CCPR5L = 0xf4;
            }    
            else{   // These compare values is 3/4 of a half PWM period 
                CCPR5H == 0x01;         // 0x177 = 375
                CCPR5L == 0x77;
            }
            
            n++;
            if(n >= 2*SAMPLES){ // 8 PWM measurements are made during one i2c request
                n = 0;
                
                T3CONbits.TMR3ON = 0;   // disable timer 3
                TMR3H = 0;
                TMR3L = 0;
                PIE4bits.CCP5IE = 0;
                
                TRISCbits.RC2 = 1;          // disable output driver/P1A
            }
            
            PIR4bits.CCP5IF = 0; 
    }
    /*else if(PIR1bits.RC1IF == 1){
        read_eusart();
    }*/
    // Timer2 interrupt
    /*else if(PIR1bits.TMR2IF == 1){
        Timer2_interrupt();         // Can be found in PWMsteering
    }*/
    // Timer4 interrupt
    else if(PIR5bits.TMR4IF == 1){
        //Timer4_interrupt();         // Can be found in PWMsteering
        PIR5bits.TMR4IF = 0;
        TIMER4_UPDATE = 1;
        /*
        I2C_COUNTER++;
        uint16_t limit;
        
        
        if((ADCON0bits.GODONE == 0)  &&  (ADC_CALCULATION == 0)){
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
        }*/
    
    }
    // ADC interrupt
    else if(PIR1bits.ADIF == 1){
        //ADC_interrupt();            // Can be found in ADChandler
        ADC_CALCULATION = 1;
        PIR1bits.ADIF = 0;          // Clear ADC interrupt flag
    }



}

void high_priority interrupt High_ISR(void){
    // I2C interrupt
    if(PIR3bits.SSP2IF == 1){
        
        //I2C_interrupt();
        
        
        
    PIR3bits.SSP2IF = 0;            // Clear interrupt flag
    static uint8_t i2c_buffer;
    //static uint32_t first_bytes = 0x11223344;
    //static uint16_t second_bytes = 0x5566;
    static unsigned char data_transmit[7] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x11, 0x61};
    static uint8_t element = 0;
    
           
    //while(SSP2STATbits.BF);
    
    if((SSP2STATbits.D_A == 0) && (!SSP2STATbits.R_W)){
        i2c_buffer = SSP2BUF;
        //array[i++] = i2c_buffer;
    }
    else if((!SSP2STATbits.R_W) && (i2c_buffer != 0x01)){    // Matching address with read write cleared
        i2c_buffer = SSP2BUF;   // Reads the received address which clear BF
        //array[i++] = i2c_buffer;
    }
    /*else if(i2c_buffer == 0x01 && (!SSP2STATbits.R_W)){         // Electrotechs start command  
        i2c_buffer = SSP2BUF;   // Reads the received address which clear BF
        //array[i++] = i2c_buffer;
        I2C_Slave_measure();
        I2C_REQUEST = 1;            // This bit is cleared in pwm_amplitude() (ADC_nerstcell();)
        I2C_COUNTER = 0;
    }*/
    else if(SSP2STATbits.R_W){           // Check Read write bit
        
        if(!SSP2STATbits.D_A){
            i2c_buffer = SSP2BUF;   // Reads the received address which clear BF
            //array[i++] = i2c_buffer;
        }
        //I2C_Slave_Send('a');
        //I2C_Slave_Send('b');
        if(!SSP2CON2bits.ACKSTAT){//VALID_DATA == 0 && !SSP2CON2bits.ACKSTAT){
            
            if(element <=5){
                SSP2BUF = data_transmit[element++];
                /*if(element == 6){
                    SSP2CON1bits.SSPEN = 0;
                       while(1){
                            ;
                        }
                }*/
                
            }

            
        }
        
    }
    
    // Power up circuit after sleep
    if(LATCbits.LATC1 == 1){
        LATCbits.LATC1 = 0;         // Power up circuit
        
        PIE1bits.ADIE = 1;              // disable ADC interrupt
        PIE5bits.TMR4IE = 1;            // disable timer4 interrupt
        PIE4bits.CCP5IE = 1;            // disable compare interrupt
        //PIE4bits.CCP3IE = 1;            // disable compare interrupt
    }
    
    // Checks for a stop bit
    if(SSP2STATbits.P == 1){
        if(element > 1){
            VALID_DATA = 1;
        }
        element = 0;
        // This data is sent on next i2c request                
        data_transmit[0] = (O2_PERCENT >> 8);
        data_transmit[1] = O2_PERCENT;
                    
        data_transmit[2] = (LSU_TEMPERATURE >> 8);
        data_transmit[3] = LSU_TEMPERATURE;
                    
        data_transmit[4] = (NERST_VOLTAGE >> 8);
        data_transmit[5] = NERST_VOLTAGE;
    }
    
    SSP2CON1bits.CKP = 1;           // release clock to enable master to clock out information
       
       
    }
}



