#include "config.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stdlib.h"
#include "stdio.h"
#include "cmyserial.h"
#include "MoveBase.h"

//DC_Motor_Pin_init
Gpio_pin_parameter m_FR_Port_L;
Gpio_pin_parameter m_SV_Port_L;
Gpio_pin_parameter m_EN_Port_L;
Gpio_pin_parameter m_BK_Port_L;
Gpio_pin_parameter m_PG_Port_L;

Gpio_pin_parameter m_FR_Port_R;
Gpio_pin_parameter m_SV_Port_R;
Gpio_pin_parameter m_EN_Port_R;
Gpio_pin_parameter m_BK_Port_R;
Gpio_pin_parameter m_PG_Port_R;

Gpio_pin_parameter m_IR_Port_L;
Gpio_pin_parameter m_IR_Port_R;
Gpio_pin_parameter m_Relay_AC;

Gpio_pin_parameter m_Startup_Port;

Gpio_pin_parameter m_PowerRelay;

Gpio_pin_parameter m_SoftStop_key;

Gpio_pin_parameter m_Temp_busPin;

Gpio_pin_parameter m_Voltage_Pin;

Gpio_pin_parameter m_Elect_Pin;

Gpio_pin_parameter m_Elevator_UP;
Gpio_pin_parameter m_Elevator_DOWN;
Gpio_pin_parameter m_Elevator_UP_Limit;
Gpio_pin_parameter m_Elevator_DOWN_Limit;

Gpio_pin_parameter m_LED_Port;
Gpio_pin_parameter m_B_STOP; //抱死

//off on state
boolean UP_ON_OFF;

void MoveBasePinDefine(void)
{
	//左 电机引脚
	m_FR_Port_L.m_gpio = GPIOE;
	m_FR_Port_L.m_pin  = GPIO_Pin_9;

	m_SV_Port_L.m_gpio = GPIOA;
	m_SV_Port_L.m_pin  = GPIO_Pin_2;

	m_EN_Port_L.m_gpio = GPIOE;
	m_EN_Port_L.m_pin  = GPIO_Pin_7;

	m_BK_Port_L.m_gpio = GPIOB;
	m_BK_Port_L.m_pin  = GPIO_Pin_1;

	m_PG_Port_L.m_gpio = GPIOA;
	m_PG_Port_L.m_pin  = GPIO_Pin_0;

	//右 电机引脚
	m_FR_Port_R.m_gpio = GPIOE;
	m_FR_Port_R.m_pin  = GPIO_Pin_8;

	m_SV_Port_R.m_gpio = GPIOD;
	m_SV_Port_R.m_pin  = GPIO_Pin_12;

	m_EN_Port_R.m_gpio = GPIOB;
	m_EN_Port_R.m_pin  = GPIO_Pin_2;

	m_BK_Port_R.m_gpio = GPIOB;
	m_BK_Port_R.m_pin  = GPIO_Pin_0;

	m_PG_Port_R.m_gpio = GPIOC;
	m_PG_Port_R.m_pin  = GPIO_Pin_6;

	//IR自动充电 IR引脚
	m_IR_Port_L.m_gpio = GPIOE;
	m_IR_Port_L.m_pin  = GPIO_Pin_14;
	m_IR_Port_R.m_gpio = GPIOE;
	m_IR_Port_R.m_pin  = GPIO_Pin_12;
	//自动充电 继电器引脚
	m_Relay_AC.m_gpio = GPIOE;
	m_Relay_AC.m_pin  = GPIO_Pin_10;

	//开关机引脚
	m_Startup_Port.m_gpio = GPIOB;
	m_Startup_Port.m_pin  = GPIO_Pin_12;

	//总电源 继电器
	m_PowerRelay.m_gpio = GPIOE;
	m_PowerRelay.m_pin  = GPIO_Pin_3;

	//软停止按键
	m_SoftStop_key.m_gpio = GPIOB;
	m_SoftStop_key.m_pin  = GPIO_Pin_13;

	//温度传感器onewrite接口
	m_Temp_busPin.m_gpio = GPIOB;
	m_Temp_busPin.m_pin  = GPIO_Pin_14;

	//电流传感器接口 模拟量输入读取
	m_Elect_Pin.m_gpio = GPIOA;
	m_Elect_Pin.m_pin  = GPIO_Pin_7;

	//电压传感器接口 外部中断
	m_Voltage_Pin.m_gpio = GPIOC;
	m_Voltage_Pin.m_pin  = GPIO_Pin_4;
	//抱死开关
	m_B_STOP.m_gpio = GPIOC;
	m_B_STOP.m_pin = GPIO_Pin_8;

//	//升降机up down
//	m_Elevator_UP.m_gpio = GPIOA;
//	m_Elevator_UP.m_pin  = GPIO_Pin_8;

//	m_Elevator_DOWN.m_gpio = GPIOC;
//	m_Elevator_DOWN.m_pin  = GPIO_Pin_9;

//	m_Elevator_UP_Limit.m_gpio = GPIOA;
//	m_Elevator_UP_Limit.m_pin  = GPIO_Pin_12;

//	m_Elevator_DOWN_Limit.m_gpio = GPIOA;
//	m_Elevator_DOWN_Limit.m_pin  = GPIO_Pin_11;

	//LED指示灯引脚
	m_LED_Port.m_gpio = GPIOE;
	m_LED_Port.m_pin  = GPIO_Pin_0;
}

void RCC_SystemClkInit(void)
{
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	while( RCC_WaitForHSEStartUp() != SUCCESS );
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	RCC_HCLKConfig(RCC_HCLK_Div1);//AHB 168M
	RCC_PCLK1Config(RCC_HCLK_Div4);//APB1 42M
	RCC_PCLK2Config(RCC_HCLK_Div2);//APB2 84M
	RCC_PLLConfig(RCC_PLLSource_HSE, PLL_M, PLL_N, PLL_P, PLL_Q);
	RCC_PLLCmd(ENABLE);
	while(RCC_GetSYSCLKSource() != 0x08);	
	RCC_HSICmd(DISABLE);
}


uint16_t ConfigClass_readUSART(USART_TypeDef* USARTx)
{

	if( USART_GetFlagStatus(USARTx,USART_FLAG_RXNE))
		return USART_ReceiveData(USARTx);
	else
		return 0xFFFF;
}


void ConfigClass_USART1Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9, GPIO_AF_USART1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE ,ENABLE);
	USART_Cmd(USART1, ENABLE);//使能串口

	/*DMA设置*/
	//USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	//ConfigClass_USART1DMAInit();
}
void ConfigClass_USART2Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	//串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6, GPIO_AF_USART2);

	//USART6端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//USART6 初始化设置
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE ,ENABLE);

	USART_Cmd(USART2, ENABLE);//使能串口
}


void ConfigClass_USART3Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

	//串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9, GPIO_AF_USART3);

	//USART6端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD,&GPIO_InitStructure);

	//USART6 初始化设置
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE ,ENABLE);

	USART_Cmd(USART3, ENABLE);//使能串口
}


void ConfigClass_USART4Init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);

	//串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11, GPIO_AF_UART4);

	//USART6端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	//USART6 初始化设置
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_ITConfig(UART4, USART_IT_RXNE ,ENABLE);

	USART_Cmd(UART4, ENABLE);//使能串口
}

void USART1_IRQHandler(void)
{
	if (USART_GetFlagStatus(USART1, USART_FLAG_PE) != RESET){
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1, USART_FLAG_PE);
	}  
	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET){
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1, USART_FLAG_ORE);
	}
	if (USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET){
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1, USART_FLAG_FE);
	}
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
		//ConfigClass_readUSART(USART1);
		//CMySerial_addCmdtoOrignal_buff(&m_stepperControl.m_ccm, ConfigClass_readUSART(USART1));
		USART_ClearFlag(USART1, USART_FLAG_RXNE);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

/*对外串口*/
void USART2_IRQHandler(void)
{
	if (USART_GetFlagStatus(USART2, USART_FLAG_PE) != RESET){
		USART_ReceiveData(USART2);
		USART_ClearFlag(USART2, USART_FLAG_PE);
	}  
	if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET){
		USART_ReceiveData(USART2);
		USART_ClearFlag(USART2, USART_FLAG_ORE);
	}
	if (USART_GetFlagStatus(USART2, USART_FLAG_FE) != RESET){
		USART_ReceiveData(USART2);
		USART_ClearFlag(USART2, USART_FLAG_FE);
	}
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
		//CMySerial_addCmdtoOrignal_buff(&m_stepperControl.m_ccm2, ConfigClass_readUSART(USART2));
//		m_message.m_transmissionBuffStruct.m_UWBUARTtoWIFIandW5500.m_currentReceiveTime = millis();
		USART_ClearFlag(USART2, USART_FLAG_RXNE);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}

void USART3_IRQHandler(void)
{
	if (USART_GetFlagStatus(USART3, USART_FLAG_PE) != RESET){
		USART_ReceiveData(USART3);
		USART_ClearFlag(USART3, USART_FLAG_PE);
	}  
	if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET){
		USART_ReceiveData(USART3);
		USART_ClearFlag(USART3, USART_FLAG_ORE);
	}
	if (USART_GetFlagStatus(USART3, USART_FLAG_FE) != RESET){
		USART_ReceiveData(USART3);
		USART_ClearFlag(USART3, USART_FLAG_FE);
	}
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){
		CMySerial_addCmdtoOrignal_buff(&MoveBase.m_ccm3, ConfigClass_readUSART(USART3));
		//CMySerial_addCmdtoOrignal_buff(&m_stepperControl.m_ccm2, ConfigClass_readUSART(USART2));
//		m_message.m_transmissionBuffStruct.m_UWBUARTtoWIFIandW5500.m_currentReceiveTime = millis();
		USART_ClearFlag(USART3, USART_FLAG_RXNE);
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}



void UART4_IRQHandler(void)
{
	if (USART_GetFlagStatus(UART4, USART_FLAG_PE) != RESET){
		USART_ReceiveData(UART4);
		USART_ClearFlag(UART4, USART_FLAG_PE);
	}  
	if (USART_GetFlagStatus(UART4, USART_FLAG_ORE) != RESET){
		USART_ReceiveData(UART4);
		USART_ClearFlag(UART4, USART_FLAG_ORE);
	}
	if (USART_GetFlagStatus(UART4, USART_FLAG_FE) != RESET){
		USART_ReceiveData(UART4);
		USART_ClearFlag(UART4, USART_FLAG_FE);
	}
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET){
		CMySerial_addCmdtoOrignal_buff(&MoveBase.m_ccm, ConfigClass_readUSART(UART4));
//		m_message.m_transmissionBuffStruct.m_UWBUARTtoWIFIandW5500.m_currentReceiveTime = millis();
		USART_ClearFlag(UART4, USART_FLAG_RXNE);
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	}
}





void myprintfUSART1( const char* fmt, ... )
{
	static char buf[256];
	va_list ap;
	va_start( ap, fmt );
	vsprintf( buf, fmt, ap );
	va_end( ap );
	for(int i = 0; i < strlen(buf); i++){
		USART_SendData(USART1, buf[i]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);		
	}
}

void myprintfUSART2( const char* fmt, ... )
{
	static char buf[256];
	va_list ap;
	va_start( ap, fmt );
	vsprintf( buf, fmt, ap );
	va_end( ap );
	for(int i = 0; i < strlen(buf); i++){
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);	
		USART_SendData(USART2, buf[i]);			
	}
}


void myprintfUSART3( const char* fmt, ... )
{
	static char buf[256];
	va_list ap;
	va_start( ap, fmt );
	vsprintf( buf, fmt, ap );
	va_end( ap );
	for(int i = 0; i < strlen(buf); i++){
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);	
		USART_SendData(USART3, buf[i]);
			
	}
}

void myprintfUSART4( const char* fmt, ... )
{
	static char buf[256];
	va_list ap;
	va_start( ap, fmt );
	vsprintf( buf, fmt, ap );
	va_end( ap );
	for(int i = 0; i < strlen(buf); i++){
		while(USART_GetFlagStatus(UART4,USART_FLAG_TC)!=SET);	
		USART_SendData(UART4, buf[i]);			
	}
}


void mySerialWriteUSART1( const byte* sendBuff, int sendLen )
{
	for( int i = 0; i < sendLen; i++ ){
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
		USART_SendData(USART1, sendBuff[i]);		
	}
}

void mySerialWriteUSART2( const byte* sendBuff, int sendLen )
{
	for( int i = 0; i < sendLen; i++ ){
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);
		USART_SendData(USART2, sendBuff[i]);		
	}
}

void mySerialWriteUSART3( const byte* sendBuff, int sendLen )
{
	for( int i = 0; i < sendLen; i++ ){
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);
		USART_SendData(USART3, sendBuff[i]);		
	}
}

void mySerialWriteUSART4( const byte* sendBuff, int sendLen )
{
	for( int i = 0; i < sendLen; i++ ){
		while(USART_GetFlagStatus(UART4,USART_FLAG_TC)!=SET);
		USART_SendData(UART4, sendBuff[i]);		
	}
}



void pinInputModeInit(Gpio_pin_parameter* Gpio_pin_para)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	if( Gpio_pin_para->m_gpio == GPIOA )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	else if( Gpio_pin_para->m_gpio == GPIOB )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	else if( Gpio_pin_para->m_gpio == GPIOC )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	else if( Gpio_pin_para->m_gpio == GPIOD )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	else if( Gpio_pin_para->m_gpio == GPIOE )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	else if( Gpio_pin_para->m_gpio == GPIOF )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	else if( Gpio_pin_para->m_gpio == GPIOG )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	//
	GPIO_InitStructure.GPIO_Pin = Gpio_pin_para->m_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(Gpio_pin_para->m_gpio, &GPIO_InitStructure);
}

void pinOutputModeInit( Gpio_pin_parameter* Gpio_pin_para )
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	if( Gpio_pin_para->m_gpio == GPIOA )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	else if( Gpio_pin_para->m_gpio == GPIOB )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	else if( Gpio_pin_para->m_gpio == GPIOC )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	else if( Gpio_pin_para->m_gpio == GPIOD )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	else if( Gpio_pin_para->m_gpio == GPIOE )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	else if( Gpio_pin_para->m_gpio == GPIOF )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	else if( Gpio_pin_para->m_gpio == GPIOG )
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = Gpio_pin_para->m_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(Gpio_pin_para->m_gpio, &GPIO_InitStructure);
}

void pinDigitalWrite( Gpio_pin_parameter* Gpio_pin_para, boolean fal )
{
	if( fal == false )
		GPIO_ResetBits(Gpio_pin_para->m_gpio,Gpio_pin_para->m_pin);
	else if( fal == true )
		GPIO_SetBits(Gpio_pin_para->m_gpio,Gpio_pin_para->m_pin);
	else
		GPIO_ResetBits(Gpio_pin_para->m_gpio,Gpio_pin_para->m_pin);
}


uint8_t pinDigitalRead( Gpio_pin_parameter* Gpio_pin_para )
{
	return GPIO_ReadInputDataBit( Gpio_pin_para->m_gpio, Gpio_pin_para->m_pin );
}

void mySystem_init(void)
{

	RCC_SystemClkInit();
	//exit interrupt init
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//systick init
	Delay_Init();
	ConfigClass_USART1Init(9600);
	ConfigClass_USART3Init(9600);
	ConfigClass_USART4Init(9600);
}


void add_num_to_str_float( char* str, float x )
{
	char tmp[32];
	sprintf( tmp, " %0.2f", x );
	strcat( str, tmp );
}

void add_num_to_str_int( char* str, long x )
{
  char tmp[32];
  sprintf( tmp, " %ld", x );
  strcat( str, tmp );
}

void myprintf( const char* fmt, ... )
{
	static char buf[256];
	va_list ap;
	va_start( ap, fmt );
	vsprintf( buf, fmt, ap );
	va_end( ap );
	for(int i = 0; i < strlen(buf); i++){
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
		USART_SendData(USART1, buf[i]);
	}
}

u16 readFromUart(void)
{
	if( USART_GetFlagStatus(USART1,USART_FLAG_RXNE))
		return USART_ReceiveData(USART1);
	else
		return 0xFFFF;
}







