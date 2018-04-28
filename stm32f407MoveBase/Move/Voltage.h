#ifndef _Voltage_H
#define _Voltage_H

#include "config.h"

typedef struct CVoltage_parameter_all
{
	Gpio_pin_parameter*	m_volPin;
	float				m_Voltage;
}CVoltage_parameter;

void CVoltage_setup( CVoltage_parameter* CVoltage_para, 
					 Gpio_pin_parameter* Gpio_pin );

#endif
