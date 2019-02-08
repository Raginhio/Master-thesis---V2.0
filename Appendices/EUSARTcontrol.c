/*
 * File:   EUSART_control.c
 * Author: davrag
 *
 * Created on den 2 februari 2017, 15:46
 */


#include "EUSARTcontrol.h"


// Initializing EUSART
void init_eusart(void) {

    // Baud rate generator 
    if(HF_CLOCK == 16000000){
        TXSTAbits.BRGH = 0;     // Low speed asynchronous mode
        BAUDCONbits.BRG16 = 0;  // 8-bit baud rate is used
        SPBRG1        = HF_CLOCK/9600/64 - 1;        //25;     // Baud rate set to 9600 when BRGH = 0
    }
    else if(HF_CLOCK == 1000000){
        TXSTAbits.BRGH = 1;     // High speed asynchronous mode
        BAUDCONbits.BRG16 = 1;  // 8-bit baud rate is used
        SPBRG1        = 25;     // Baud rate set to 9600 when BRGH = 0
    }
    else if(HF_CLOCK == 2000000){
        TXSTAbits.BRGH = 1;     // Low speed asynchronous mode
        BAUDCONbits.BRG16 = 1;  // 8-bit baud rate is used
        SPBRG1        = HF_CLOCK/9600/4 - 1;        //25;     // Baud rate set to 9600 when BRGH = 0
        SPBRGH1 = 0;
    }
    
    // Mainly transmit configurations
    TRISCbits.RC6 = 1;      // TX1
    TRISCbits.RC7 = 1;      // RX1
    ANSELCbits.ANSC7 = 0;
    
    TXSTAbits.SYNC = 0;     // Asynchronous mode
    RCSTAbits.SPEN = 1;     // Serial port enabled
    
    TXSTAbits.TXEN = 1;     // Transmit enable bit
    
    // Receive configurations
    PIE1bits.RC1IE = 1;     // Enable receive interrupt
    PIR1bits.RC1IF = 0;
    RCSTAbits.CREN = 1;     // Enable receiver
        
}

// Function to convert integer to string and send in through eusart
void send_eusart(int value){
  
    int temp_value, element=0;
    char str[5] = "\0";
    
    // Converts int to char(string)
    while(value > 0){
    temp_value = value;
    temp_value /= 10;
    temp_value *= 10;
    temp_value = value - temp_value;
    
    str[element++] = temp_value + '0';
    value /= 10;
    }
    
    // sends string through TX1
    while(element >= 0){
        while(PIR1bits.TX1IF == 0){    // Wait for transmission to complete
            ;
        }
        TXREG1  =   str[element--];
    }

    while(PIR1bits.TX1IF == 0){    // Wait for transmission to complete
        ;
    }    
    TXREG1  =   '\r';
    while(PIR1bits.TX1IF == 0){    // Wait for transmission to complete
        ;
    }
    TXREG1  =   '\n';
    return;
}

void send_eusart_string(unsigned char send[]){
    
    int element = 0;
    // Getting string size;
    while(send[element++] != '\0'){
        ;
    }
    
    // Sending string through TX1
    while(element >= 0){
        while(PIR1bits.TX1IF == 0){    // Wait for transmission to complete
            ;
        }    
        TXREG1  =   send[element--];
    }
    
    while(PIR1bits.TX1IF == 0){    // Wait for transmission to complete
        ;
    }
}

void read_eusart(void){
    char symbol;
    static int DAC_voltage = 2048;
    int error_flag;
    
    error_flag = BAUDCON1;
    
    symbol = RCREG1;
    while((PIR1bits.RC1IF == 1) && (BAUDCON1bits.RCIDL == 0)){
        ;
    }
    
    TXREG1  =   symbol;
    while(PIR1bits.TX1IF == 0){    // Wait for transmission to complete
        ;
    }
    
    if(symbol == '+'){
        DAC_voltage += 10;
        I2C_Master_DAC(DAC_voltage);
    }
    else if(symbol == '-'){
        DAC_voltage -= 10;
        I2C_Master_DAC(DAC_voltage);
    }
    else if(symbol == '5'){         // Nerst voltage
        ADCON0bits.CHS = 0b01101;   // Analog channel selected to AN13(RB5)
        ADCON0bits.GODONE = 1;      // Starts a new conversion
    }
    else if(symbol == '3'){
        ADCON0bits.CHS = 0b01001;   // Analog channel selected to AN9(RB3)
        ADCON0bits.GODONE = 1;      // Starts a new conversion
    }
    else if(symbol == 'f'){         // FVR
        ADCON0bits.CHS = 0b11111;   // Analog channel selected to AN13(RB5)
        ADCON0bits.GODONE = 1;      // Starts a new conversion
    }
    else if(symbol == '4'){
        uint32_t pwm_volt;
        pwm_volt = pwm_amplitude_value;
        pwm_volt *= 3300;
        pwm_volt /= 1024;
        send_eusart_string("\n\r:Vm ni edutilpma MWP ehT\n\r");
        send_eusart(pwm_volt);
    }
    else if(symbol == 'i'){
        send_eusart_string("\n\r:si egatlov CAD ehT\n\r");
        send_eusart(DAC_voltage);        
    }
    else{
        send_eusart_string("\n\r!dnammoc nwonknU\n\r");
        send_eusart_string("\n\r:snoitcnuf fo tsiL");
        send_eusart_string("\n\redutilpma MWP=4");
        send_eusart_string("\n\regatlov tsreN=5");
        send_eusart_string("\n\rV840.2 FVR=f");
        send_eusart_string("\n\reulav CAD=i");
        send_eusart_string("\n\rCAD ot 01+/-=+/-");
    }
    /*while(PIR1bits.TX1IF == 0){    // Wait for transmission to complete
        ;
    }    
    TXREG1  =   '\r';
    while(PIR1bits.TX1IF == 0){    // Wait for transmission to complete
        ;
    }
    TXREG1  =   '\n';*/
}
