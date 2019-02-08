/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

 
#ifndef ADC_HANDLER_H_INCLUDED
#define	ADC_HANDLER_H_INCLUDED

#include "main.h"



// function defining
void init_ADC(void);
void pwm_amplitude(int adc_value);
void ADC_interrupt(void);
void ADC_temperature(void);
void ADC_nerstcell(void);
void ADC_Ipmeas(void);
void init_FVR(void);
void ADC_fvr(void);
void ADC_virtual_ground(void);
void Compare5_interrupt(void);
void O2_calculation(uint16_t adc_value, uint8_t decider);
void measure_sensor(void);

#endif	/* XC_HEADER_TEMPLATE_H */

