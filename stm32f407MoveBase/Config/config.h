#ifndef _CONFIG_H
#define _CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_conf.h"
#include "delay.h"
#include "spi.h"
#include "stdarg.h"
#include "delay.h"
//#include "DW1000.h"
#include "commen.h"

/*PLL锁相环设置*/
#define PLL_M      8
#ifdef BOOCASE_16M
	#define PLL_N      168//336//
#else
	 #define PLL_N     336//
#endif
#define PLL_P      2
#define PLL_Q      4

//system clk init
void mySystem_init(void);

void myprintf( const char* fmt, ... );

void ConfigClass_USART1Init(uint32_t baud);
void ConfigClass_USART2Init(uint32_t baud);
void ConfigClass_USART3Init(uint32_t baud);
void ConfigClass_USART4Init(uint32_t baud);
void myprintfUSART1( const char* fmt, ... );
void myprintfUSART2( const char* fmt, ... );
void myprintfUSART3( const char* fmt, ... );
void myprintfUSART4( const char* fmt, ... );
void mySerialWriteUSART1( const byte* sendBuff, int sendLen );
void mySerialWriteUSART2( const byte* sendBuff, int sendLen );
void mySerialWriteUSART3( const byte* sendBuff, int sendLen );
void mySerialWriteUSART4( const byte* sendBuff, int sendLen );
//void UART1_Init(void);
u16 readFromUart(void);

//void UART2_Init(void);
u16 readFromUart2(void);
void writeToUart2(u8 data);

void add_num_to_str_int( char* str, long x );
void add_num_to_str_float( char* str, float x );

//Car_parameter
#define m_WHEEL_R     60//120/2; // mm, ban jing
#define m_WHEEL_D     386//386; // mm, di pan

//off on state
extern boolean UP_ON_OFF;

//gpio struct
typedef struct Gpio_pin_parameter_all
{
	GPIO_TypeDef*	m_gpio;//huoer
	uint16_t		m_pin;
}Gpio_pin_parameter;

//DC_Motor_Pin_init
extern Gpio_pin_parameter m_FR_Port_L;
extern Gpio_pin_parameter m_SV_Port_L;
extern Gpio_pin_parameter m_EN_Port_L;
extern Gpio_pin_parameter m_BK_Port_L;
extern Gpio_pin_parameter m_PG_Port_L;

extern Gpio_pin_parameter m_FR_Port_R;
extern Gpio_pin_parameter m_SV_Port_R;
extern Gpio_pin_parameter m_EN_Port_R;
extern Gpio_pin_parameter m_BK_Port_R;
extern Gpio_pin_parameter m_PG_Port_R;

extern Gpio_pin_parameter m_IR_Port_L;
extern Gpio_pin_parameter m_IR_Port_R;
extern Gpio_pin_parameter m_Relay_AC;

extern Gpio_pin_parameter m_Startup_Port;

extern Gpio_pin_parameter m_PowerRelay;

extern Gpio_pin_parameter m_SoftStop_key;

extern Gpio_pin_parameter m_Temp_busPin;

extern Gpio_pin_parameter m_Voltage_Pin;

extern Gpio_pin_parameter m_Elect_Pin;

extern Gpio_pin_parameter m_Elevator_UP;
extern Gpio_pin_parameter m_Elevator_DOWN;
extern Gpio_pin_parameter m_Elevator_UP_Limit;
extern Gpio_pin_parameter m_Elevator_DOWN_Limit;

extern Gpio_pin_parameter m_LED_Port;
extern Gpio_pin_parameter m_B_STOP;

void MoveBasePinDefine(void);
void pinInputModeInit(Gpio_pin_parameter* Gpio_pin_para);
void pinOutputModeInit(Gpio_pin_parameter* Gpio_pin_para);
void pinDigitalWrite( Gpio_pin_parameter* Gpio_pin_para, boolean fal );
uint8_t pinDigitalRead( Gpio_pin_parameter* Gpio_pin_para );

#ifdef __cplusplus
}
#endif

#endif
