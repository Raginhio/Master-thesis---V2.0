/*
 * File:   I2C_handler.c
 * Author: davrag
 *
 * Created on den 9 februari 2017, 09:00
 */


#include <pic18f26k22.h>

#include "I2Chandler.h"


void init_I2C(void) {
    
    //Configuration for I2C Master on the first bus (RC3 & RC4)  
    
    //PIE1bits.SSP1IE = 1;            // Enable interrupt for I2C_1 bus
    //SSP1CON3bits.SCIE   = 1;        // enable interrupt on  detection of start or restart conditions
    
    SSP1CON1bits.SSPM = 0b1000;     // I2C master mode selected
    SSP1CON1bits.SSPEN = 1;
    SSP1CON2bits.SEN = 1;         // Enable I2C
    SSP1CON2 = 0;
    SSP1ADD = HF_CLOCK/100000/4 - 1;  //0x27; // Baud rate are set to 100 kHz when FOSC = 16MHz
        if(HF_CLOCK == 1000000){
            SSP1ADD = 0x03;
        }
    SSP1STAT = 0;
    TRISCbits.RC3 = 1;              // SDA1 and SCL1 are set as input pins
    TRISCbits.RC4 = 1;              // RC4 = SDA1,  RC3 = SCL1
    ANSELCbits.ANSC3 = 0;
    ANSELCbits.ANSC4 = 0; 
}

void init_I2C_Slave(void){
    // Configuring Pins for I2C_2 bus
    TRISBbits.RB1 = 1;              // SDA1 and SCL1 are set as input pins
    TRISBbits.RB2 = 1;              // RB1 = SDA1,  RB2 = SCL1
    ANSELBbits.ANSB1 = 0;           
    ANSELBbits.ANSB2 = 0; 
            
    // Configuring I2C slave
    SSP2STAT = 0x80;                // slew rate control disable    
    //SSP2ADD = 0x06;                 // 7-bit address LSB is unused
    SSP2ADD = 0x30;                 // 7-bit address LSB is unused
    SSP2CON1 = 0;
    SSP2CON1bits.SSPM = 0b0110;     // I2C slave mode 7-bit address
    
    SSP2STATbits.SMP = 1;   // Slew rate control disable for standard speed mode
    SSP2STATbits.CKE = 0;   // Enable input logic
    
    SSP2CON3bits.SDAHT = 1; // 300 ns
    SSP2CON3bits.BOEN = 1;
    SSP2CON3bits.PCIE = 1;      // interrupt on stop bit
    
    SSP2CON2bits.SEN = 0;           // Clock stretching disable
    //SSP2CON2bits.GCEN = 1;
    SSP2CON1bits.CKP = 1;
    SSP2CON1bits.SSPEN = 1;         // Enables the serial ports
      
    // Enable interrupt
    PIE3bits.SSP2IE = 1;    // Enables interrupt on I2C slave
}

// voltage should be a 12-bit value defining the voltage between 0-3.3V
void I2C_Master_DAC(unsigned voltage){
    
    uint8_t second_byte = (voltage & 0xff);         // the last 8-bits in the 12-bit voltage value
    uint8_t first_byte = ((voltage>>8) & 0x0f);     // First 4-bits indicates reserved and mode selection bits. 
                                                    // Last 4 bits is the MSBs for the 12-bit voltage value
    
    
    I2C_Master_Wait(); //Transmit is in progress
    SSP1CON2bits.SEN = 1;           // Generates a start condition

    I2C_Master_Write(0b00011110);   // DAC address plus write
    I2C_Master_Write(first_byte);         // 16-bit data are to be transmit. The first 4 bits will be 0 for reserved and mode selection
    I2C_Master_Write(second_byte);         // The remaining 8-bits data are transmitted
    I2C_Master_Wait(); //Transmit is in progress
    SSP1CON2bits.PEN = 1;           // Generate a stop bit
    I2C_Master_Wait(); //Transmit is in progress
}

void I2C_Master_Write(unsigned d)
{
    I2C_Master_Wait();
    SSP1BUF = d;
}

void I2C_Master_Wait()
{
    while ((SSP1STAT & 0x04) || (SSP1CON2 & 0x1F));
}




void I2C_Slave_measure(void){

        SSP2CON1bits.CKP = 1;
        
        ADCON0bits.CHS = 0b01101;   // Nerst voltage channel
        ADCON0bits.GODONE = 1;      // Starts conversion
        
        //VALID_DATA = 1;
}