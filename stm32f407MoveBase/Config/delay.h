#ifndef _DELAY_H
#define _DELAY_H

#ifdef __cplusplus  
extern "C" {
#endif  

#include "stm32f4xx.h"


void Delay_Init(void);
void delay( u32 delayms );//delay ms
void delay_us( u32 delayus );

u32 millis(void);//same to arduino millis()

#ifdef __cplusplus  
}  
#endif
	
#endif
