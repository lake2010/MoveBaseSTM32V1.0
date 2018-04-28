#ifndef _DS18B20_H
#define _DS18B20_H
#include "config.h"

#define DS18B20_busPinOutLow(DSGPIOx, DSGPIO_Pin)	GPIO_ResetBits(DSGPIOx, DSGPIO_Pin)
#define DS18B20_busPinOutHigh(DSGPIOx, DSGPIO_Pin)	GPIO_SetBits(DSGPIOx, DSGPIO_Pin)
#define DS18B20_busPinIn(DSGPIOx, DSGPIO_Pin)		GPIO_ReadInputDataBit(DSGPIOx, DSGPIO_Pin)

typedef struct DS18B20_parameter_all
{
	Gpio_pin_parameter*	m_busPin;
	unsigned char		m_sensorNum;//预设链接传感器数量
	unsigned char		DS18B20_ID[8][8];//ID 检测存储区
	unsigned char		DS18B20_Num;//实际检测到的传感器数量

}DS18B20_parameter;

u8 DS18B20_setup(	DS18B20_parameter*	DS18B20_para, 
					Gpio_pin_parameter*	Gpio_pin, 
					unsigned char		SensorNum);
void DS18B20_searchRom( DS18B20_parameter* DS18B20_para );
float DS18B20_getTemp( DS18B20_parameter* DS18B20_para, u8 i);

#endif // !_DS18B20_H
