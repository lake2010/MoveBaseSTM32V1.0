#include "LED.h"

void CLED_setup( CLED_parameter* CLED_para, Gpio_pin_parameter* Gpio_pin_para )
{
	CLED_para->m_ledPin = Gpio_pin_para;
	CLED_para->m_canLedBlink = true;
	CLED_para->m_ledIsOn = true;
	CLED_para->m_startTime = millis();
	CLED_para->m_ledOnTime = millis();
	pinOutputModeInit( Gpio_pin_para );
	pinDigitalWrite(Gpio_pin_para, ON);
}

void CLED_loop( CLED_parameter* CLED_para )
{
	if((millis() - CLED_para->m_startTime) < 5000)  //on startup 5s led on
		return;
	if(CLED_para->m_ledIsOn){
		if((millis() - CLED_para->m_ledOnTime) > 300){
			pinDigitalWrite(CLED_para->m_ledPin, OFF);
			CLED_para->m_ledIsOn = false;
			CLED_para->m_canLedBlink = false;
		}   
	}
	
//	else if(CLED_para->m_canLedBlink && (millis() - CLED_para->m_ledOnTime) > 600){
//			CLED_para->m_ledOnTime = millis();
//			pinDigitalWrite(CLED_para->m_ledPin, ON);
//			CLED_para->m_ledIsOn = true;  
//		}
}
