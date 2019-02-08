/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

 
#ifndef I2CHANDLER_H_INCLUDED
#define	I2CHANDLER_H_INCLUDED

#include "main.h"



// function defining
void init_I2C(void);
void init_I2C_Slave(void);
void I2C_Master_Write(unsigned d);
void I2C_Master_Wait();
void I2C_Master_DAC(unsigned voltage);


#endif	/* XC_HEADER_TEMPLATE_H */

