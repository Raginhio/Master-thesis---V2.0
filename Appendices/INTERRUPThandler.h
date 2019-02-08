/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

 
#ifndef INTERRUPT_HANDLER_H_INCLUDED
#define	INTERRUPT_HANDLER_H_INCLUDED

#include "main.h"



// function defining
void low_priority interrupt ISR();
void high_priority interrupt High_ISR(void);
void init_interrupt();
void I2C_Slave_measure(void);

#endif	/* XC_HEADER_TEMPLATE_H */

