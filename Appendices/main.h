/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

 
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded. 
#include <stdio.h>
//#include <string.h>
#include <stdint.h>
#include "PWMsteering.h"
#include "INTERRUPThandler.h"
//#include "EUSARTcontrol.h"
#include "ADChandler.h"
#include "I2Chandler.h"

//#define HF_CLOCK    2000000     // OBS init clock has to be changed manually
#define HF_CLOCK    16000000
#define SAMPLES     8              // Number of samples between each request

volatile uint16_t pwm_amplitude_value;
volatile uint8_t VALID_DATA = 0;
volatile uint8_t I2C_REQUEST = 0;
volatile uint8_t TEMPERATURE_REQUEST = 0;
volatile uint8_t I2C_COUNTER = 0;       // Used by timer4 to see when module should go to sleep
volatile uint8_t ADC_CALCULATION = 0;
volatile uint8_t TIMER4_UPDATE = 0;

// The following three globals contains data which will be transmit to the user.
volatile uint16_t O2_PERCENT = 0;
volatile uint16_t LSU_TEMPERATURE = 0;
volatile uint16_t NERST_VOLTAGE = 0;
volatile uint16_t SUPPLY_VOLTAGE = 5000;



// Clock configuration
#pragma config FOSC = INTIO7    // CLKOUT function RA6, port function RA7. RA6 will show 1/4 of the selected freq

//#pragma config WDTEN = OFF

#pragma config WDTEN = NOSLP      // Watchdog Timer Enable bits 
#pragma config WDTPS = 1024    // Watchdog Timer Postscale Select bits (1:16384)

#pragma config XINST = OFF
//#pragma config MCLRE = EXTMCLR
#pragma config LVP = OFF


// function defining
void init_default_pins(void);
void initClockFreq(void);
void init_peripherals(void);

#endif	/* XC_HEADER_TEMPLATE_H */

