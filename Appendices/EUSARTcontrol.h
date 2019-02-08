/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

 
#ifndef EUSART_CONTROL_H_INCLUDED
#define	EUSART_CONTROL_H_INCLUDED

#include "main.h"
#include <stdlib.h>



// function defining
void init_eusart(void);
void send_eusart(int value);
void send_eusart_string(unsigned char send[]);

#endif	/* XC_HEADER_TEMPLATE_H */

