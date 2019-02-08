/*
 * File:   ADC_handler.c
 * Author: davrag
 *
 * Created on den 3 februari 2017, 08:38
 */

/*
 * The sampling order during active mode will be:
 * Nerst voltage
 * FVR
 * Virtual ground
 * 61.8 omega voltage
 * PWM voltage
 * Nerst voltage
 */


//#include <pic18f26k22.h>

#include "ADChandler.h"


// Start by configuring ADC conversion on AN11 (RB4)
void init_ADC(void) {
    
    TRISBbits.RB3 = 1;
    ANSELBbits.ANSB3 = 0;   // Pin configured as analog
    
    TRISBbits.RB4 = 1;      // Disable pin output driver
    ANSELBbits.ANSB4 = 0;   // Pin configured as analog
    
    TRISBbits.RB5 = 1;      // Disable pin output driver
    ANSELBbits.ANSB5 = 0;   // Pin configured as analog
    
    // Reference pins
    TRISAbits.RA2 = 1;      // RA2 is used as Vref-
    ANSELAbits.ANSA2 = 1;
    
    TRISAbits.RA3 = 1;      // RA3 is used as Vref+
    ANSELAbits.ANSA3 = 1;
        
    ADCON2bits.ADCS = 0b101;    // FOSC/16 conversion clock selected
    
    // Reference settings.
    // On the real device Vref- and Vref+ from RA2 & RA3 will be used.
    ADCON1bits.PVCFG = 0b01;    // (0b00 = Positive vref = Vdd) (0b01 Vref+ = RA3)
    ADCON1bits.NVCFG = 0b01;    // (0b00 = negative vref = Vss (gnd)) (0b01 Vref- = RA2)
    
    ADCON0bits.CHS = 0b01011;   // Analog channel selected to AN11(RB4)
    ADCON2bits.ADFM = 1;        // Right justified result (10-bit)
    ADCON2bits.ACQT = 0b100;    // Acquisition delay set to 8 TAD
    
    ADCON0bits.ADON = 1;        // ADC is enabled
    
    // ADC interrupt is initialized in interrupt handler
    
}

// Fixed voltage reference
void init_FVR(void){
    VREFCON0bits.FVRS = 0b10;   // FVR selected to 2.048 V
    VREFCON0bits.FVREN = 1;     // FVR is enabled
}

// Temperature measurements
void pwm_amplitude(int adc_value){
    

    uint32_t resistance;
    uint32_t gain = 193;        // Actual gain is 19.3
    
    //if(PIE4bits.CCP5IE == 1){
  
        pwm_amplitude_value = adc_value;        //pwm_high - pwm_low;
        resistance = pwm_amplitude_value;       // Convert pwm_amplitude_value to 32-bit and take care of the extra *10
    
        resistance = (resistance*220000)/(1024*gain-resistance*10);
        LSU_TEMPERATURE = resistance; 
        TEMPERATURE_REQUEST = 0;    // Set in oxygen calculation
        
        //I2C_REQUEST = 0;            // This bit is set by electrotech in I2C_interrupt
        //ADCON0bits.CHS = 0b01101;   // ADC nerst voltage
        //ADCON0bits.GODONE = 1;
    

}

void ADC_interrupt(void){
        PIR1bits.ADIF = 0;          // Clear ADC interrupt flag
        if(ADCON0bits.CHS == 0b01011){      // AN11 (RB4) temperature control cell
            ADC_temperature();
        }
        else if(ADCON0bits.CHS == 0b01101){     // AN13 (RB5) Nerst cell
            ADC_nerstcell();
        }
        else if(ADCON0bits.CHS == 0b11111){     // FVR
            ADC_fvr();
        }
        else if(ADCON0bits.CHS == 0b01001){     // AN9 (RB3) 61.8 omega
            ADC_Ipmeas();
        }
        else if(ADCON0bits.CHS == 0b00001){    // AN1 (RA1) IPN virtual ground
            ADC_virtual_ground();
        }
}

void ADC_temperature(void){
        
    static uint16_t pwm_high;
    static uint16_t pwm_low;
    static uint8_t highORlow = 0;
    static uint8_t m = 0;
    int AN11;
    AN11 = ADRESL;
    AN11 = ADRESL + (ADRESH<<8);

    if(highORlow == 0){
        pwm_high += AN11;
        highORlow = 1;
    }
    else{
        m++;
        pwm_low += AN11;
        highORlow = 0;
    }  
    
    if(m >= SAMPLES){
        m = 0;
        
        uint32_t resistance;
        uint32_t gain = 193;        // Actual gain is 19.3
    
    //if(PIE4bits.CCP5IE == 1){
  
        pwm_amplitude_value = pwm_high - pwm_low;
        pwm_amplitude_value /= 8;
        pwm_high = 0;
        pwm_low = 0;
        resistance = pwm_amplitude_value;       // Convert pwm_amplitude_value to 32-bit and take care of the extra *10
    
        resistance = (resistance*220000)/(1024*gain-resistance*10);
        LSU_TEMPERATURE = resistance; 
        I2C_REQUEST = 1;            // Set so oxygen will be calculated on same request
        TEMPERATURE_REQUEST = 0;    // Set in oxygen calculation
        
        //pwm_amplitude(AN11);
    }    //send_eusart(AN11);

        //ADCON0bits.GODONE = 1;      // Starts new conversion    
}

void ADC_nerstcell(void){
    //ADCON0bits.CHS = 0b01011;   // Analog channel selected to AN11(RB4)
    static int16_t DAC_volt = 2048;
    static uint16_t mean_nerst = 0;
    uint16_t AN13;
    uint16_t temp;          // used as a buffer for math instructions
    static uint8_t alternate = 0;
    static uint8_t l = 0;
    
    // Variables used for PI controller
    static int16_t integral = 0, prev_error = 0;
    int16_t max_integral = 512;
    int16_t setpoint = 28862, error, temp_integral, derivative;    // 28800 = 450 * 64
    int16_t ki = 1, kp = 1, kd = 0;
    
    
    l++;
    
    AN13 = ADRESL;
    AN13 = ADRESL + (ADRESH<<8);
    //AN13 = AN13*SUPPLY_VOLTAGE;
    //AN13 /= 5120;      // 5120 = 1024 * 5
    temp = SUPPLY_VOLTAGE/80;       // 80 = 5120/64
    AN13 = AN13*temp;
    AN13 = AN13;         // because 80 = 5120/64
    
    mean_nerst += AN13/64;
    if(l >= SAMPLES){
        mean_nerst /= SAMPLES;
        NERST_VOLTAGE = mean_nerst;           // 16-bit global sent to master
        mean_nerst = 0;
        l = 0;
    }
    
    if(AN13 > 60000){           // set limits on AN13 because of overflow when mixed sign and unsigned
        error = setpoint - 60000;
    }
    else{
        error = setpoint - AN13;
    }
    
    derivative = prev_error - error;
    integral = integral + error;
    prev_error = error;
    
    if(integral > max_integral){
        integral = max_integral;
    }
    else if(integral < -max_integral){
        integral = -max_integral;
    }
    
    /* Best 2 options so far:
     * PI ->  I = 512,   P = 64
     * PID -> P = 55,  I = 415,  D = 1880
     */
    
    temp_integral = integral / max_integral;         // 512 good value
    error /= 64;                            // 64 good value        50
    derivative /= 1880;                        // 1880 is good value
    
    if(derivative > 4){                    // Rate limiter for the Derivativ part of the PID controller
        derivative = 4;
    }
    else if(derivative < -4){
        derivative = -4;
    }
    
    
    DAC_volt += kp*error + temp_integral*ki+ derivative*kd;
    
    // Ensures no overflow on DAC value
    if(DAC_volt >= 4095){
        DAC_volt = 4095;
    }
    else if(DAC_volt <= 0){
        DAC_volt = 0;
    }
    
    I2C_Master_DAC(DAC_volt);
    
    /*if(I2C_REQUEST == 0){
        //ADCON0bits.CHS = 0b01101;   // Nerst voltage channel
        //ADCON0bits.GODONE = 1;      // Restart conversion for nerst cell
    }*/
    if((I2C_REQUEST == 1) && (TEMPERATURE_REQUEST == 0)){
        //I2C_REQUEST = 0;            // This bit is set by electrotech in I2C_interrupt
        if(alternate == 0){
            ADCON0bits.CHS = 0b11111;   // FVR channel
            ADCON0bits.GODONE = 1;      // start conversion        
        }
        if(alternate == 1){
            ADCON0bits.CHS = 0b00001;    // AN1 (RA1) IPN virtual ground
            ADCON0bits.GODONE = 1;       // start conversion        
        }
        if(alternate == 2){
            ADCON0bits.CHS = 0b01001;   // AN9 (RB3) 61.8 omega
            ADCON0bits.GODONE = 1;      // start conversion        
        }
        
        alternate++;
        if(alternate == 4){
            alternate = 0;
        }
    }
    
    //send_eusart_string("\n\r:Vm ni egatlov tsreN ehT\n\r");
    //send_eusart(AN13);
}

void ADC_Ipmeas(void){
    
    static uint16_t mean_ipmeas = 0;
    static uint8_t i = 0;
    uint16_t AN9;
    
    i++;
    
    AN9 = ADRESL;
    AN9 = ADRESL + (ADRESH<<8);
    //AN9 = AN9*3300/1024;
    
    mean_ipmeas += AN9;
    
    if(i >= SAMPLES){
        //mean_ipmeas /= SAMPLES;
        O2_calculation(mean_ipmeas, 2);     //
        mean_ipmeas = 0;
        i = 0;
    }
     
    //ADCON0bits.CHS = 0b01101;     // AN13 (RB5) Nerst cell
    
    
    /*
    ADCON0bits.CHS = 0b01011;   // PWM voltage AN11 (RB4)
    
    T3CONbits.TMR3ON = 0;   // disable timer3
    TMR3L = 0;              // Be sure timer starts at 0
    TMR3H = 0;

    //CCPR3H = 0x01;          // 0x2ee = 750 which will be 3/4 of the PWMs duty cycle
    //CCPR3L = 0x77;          // 0x177 = 375
    CCPR5H = 0x01;          // 0x2ee = 750 which will be 3/4 of the PWMs duty cycle
    CCPR5L = 0x77;          // 0x177 = 375    
    //PIR4bits.CCP3IF = 0;    // Clear interrupt flag
    //PIE4bits.CCP3IE = 1;    // Enable compare mode for timer 3
    PIR4bits.CCP5IF = 0;    // Clear interrupt flag
    PIE4bits.CCP5IE = 1;    // Enable compare mode for timer 3
    while(TMR2 != 0);       // waits for reset on timer2
    T3CONbits.TMR3ON = 1;   // enable timer 3
    */
    
    //send_eusart_string("\n\r:si tnerruc pI ehT\n\r");
    //send_eusart(AN9);    
}

void ADC_fvr(void){
    //ADCON0bits.CHS = 0b01011;   // Analog channel selected to AN11(RB4)
    uint16_t fvr;
    static uint32_t mean_fvr = 0;
    static uint8_t k = 0;
    
    k++;
    
    fvr = ADRESL;
    fvr = ADRESL + (ADRESH<<8);
    
    mean_fvr += fvr;
    if(k>=SAMPLES){
        mean_fvr /= SAMPLES;
        mean_fvr = 2097152/mean_fvr;              // 2097152 = 1024*2048
        //fvr /= 64;
        //fvr = 32768/fvr;        // 32768 = 2097152/64

        SUPPLY_VOLTAGE = mean_fvr;
        mean_fvr = 0;
        k = 0;
    }
    //ADCON0bits.CHS = 0b00001;   // Analog channel virtual ground AN1(RA1)
    //ADCON0bits.GODONE = 1;      // Restart conversion for nerst cell
    
    //send_eusart_string("\n\r:si V840.2 RVF ehT\n\r");
    //send_eusart(fvr);
}

void ADC_virtual_ground(void){

    static uint16_t mean_virtual = 0;
    static uint8_t j = 0;
    uint16_t ipn;
    
    j++;
    
    ipn = ADRESL;
    ipn = ADRESL + (ADRESH<<8);
    
    mean_virtual += ipn;
    
    if(j >= SAMPLES){
        //mean_ipmeas /= SAMPLES;
        O2_calculation(mean_virtual, 1);     // 1 is identifyer for virtual ground
        mean_virtual = 0;
        j = 0;
    }
    
    //O2_calculation(ipn, 1);     // Virtual ground 1. Ipmeas 2 on the second input
    
    //ADCON0bits.CHS = 0b01001;   // Analog channel ipmeas AN9(RB3)
    //ADCON0bits.GODONE = 1;      // Restart conversion for ipmeas
}



void O2_calculation(uint16_t adc_value, uint8_t decider){   // Decider is used to tell if value arriving come from virtual ground or Ip meas
    static int16_t virtual_ground = 0;
    static int16_t ip_meas = 0;
    static uint8_t temp_counter = 0;
    signed int16_t temp;
    uint16_t ip_gain = 5;
    
    if(decider == 1){
        virtual_ground = adc_value;
    }
    else if(decider == 2){
        ip_meas = adc_value;
        
        temp = ip_meas - virtual_ground;
        temp /= SAMPLES;
        /*temp *=SUPPLY_VOLTAGE;
        temp *= 1000000;
        temp /=1024;
        temp /= ip_gain;        // temp is here mV over the 61.8 omega resistor
        temp /= 619;  */          // temp * 10^(-5) A
        //temp /= 128;
        //temp *= 16;             // temp = uA   ->  temp * supply / (1024 * ip_gain * 61.9) * 10^6
        temp += 0x7000;

        O2_PERCENT = temp;
        I2C_REQUEST = 0;
        temp_counter++;
        if(temp_counter >= 4){      // How often the temperature will be measured relative to the current
            temp_counter = 0;
            
            TRISCbits.RC2 = 0;          // Enable output driver/P1A
            
            TEMPERATURE_REQUEST = 1;    // Reseted in compare 5 interrupt handler
        }
    }
    

}


void measure_sensor(void){

    VALID_DATA = 0;
    SSP2CON1bits.CKP = 1;
    I2C_REQUEST = 1;            // This bit is cleared in pwm_amplitude() (ADC_nerstcell();)
    I2C_COUNTER = 0;           
    /*
    while(ADCON0bits.GODONE){
        ;
    }    
    ADCON0bits.CHS = 0b01101;   // Nerst voltage channel
    ADCON0bits.GODONE = 1;      // Starts conversion    
    */
}