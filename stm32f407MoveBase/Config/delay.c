#include "delay.h"
#include "stm32f4xx.h"
#include "config.h"
static volatile u32 MillisTime = 0;

void Delay_Init(void)
{
	//NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	SysTick->VAL = 0;//
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk; //
	SysTick->LOAD=168000; //
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //
	
	NVIC_SetPriority(SysTick_IRQn, 0x05);
}

void delay( u32 delayms )
{
	u32 StartTime = millis();
	while( millis() - StartTime <= delayms );
}

void delay_us( u32 delayus )
{
	
	u32 i = delayus*20;
	while(i--);
}

u32 millis(void)
{
	return MillisTime;
}

void SysTick_Handler(void)
{
	MillisTime++;
}
