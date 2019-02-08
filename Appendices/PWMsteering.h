/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

 
#ifndef PWMSTEERING_H_INCLUDED
#define	PWMSTEERING_H_INCLUDED

#include "main.h"



// function defining
void init_pwm(void);
void init_timer4(void);
void init_timer3(void);
void Timer2_interrupt(void);
void Timer4_interrupt(void);
void Go2Sleep(void);

#endif	/* XC_HEADER_TEMPLATE_H */

