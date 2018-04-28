#ifndef _SPI_H
#define _SPI_H

#ifdef __cplusplus  
extern "C" {  
#endif 

#include "stm32f4xx.h"
#include "config.h"
//#include "DW1000.h"
#include "commen.h"
#define SPI1_CS_High  GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define SPI1_CS_Low   GPIO_ResetBits(GPIOA, GPIO_Pin_4)
	
void SPI1_Init(void);
u8 SPI_send_and_receive_byte(u8 data);
void SPI1_End(void);
void SPI1_FastSpeed(void);
#ifdef __cplusplus  
}  
#endif 

#endif
