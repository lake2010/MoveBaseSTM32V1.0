#ifndef _LED_H
#define _LED_H

#include "config.h"

#define ON	true
#define OFF	false

typedef struct CLED_parameter_all
{
	boolean	m_canLedBlink;
	boolean m_ledIsOn;
	u32		m_ledOnTime;
	u32		m_startTime;
	Gpio_pin_parameter*	m_ledPin;
}CLED_parameter;

void CLED_setup( CLED_parameter* CLED_para, Gpio_pin_parameter* Gpio_pin_para );

void CLED_loop( CLED_parameter* CLED_para);

#endif
