#ifndef _MotorDC_H
#define _MotorDC_H
#include "config.h"


//#define m_MAX_FREQUWNCY	4000000

//#define m_MAX_SPEED_FRE	600000
extern uint16_t m_PROPORTION;

#define m_MAX_FREQUWNCY	10500000

#define m_MAX_SPEED_FRE	125000
#define m_MAX_RPM		3000////3000L * 5/4;

#define m_FOREWARD		HIGH
#define m_ENABLE		LOW
#define m_BREAK			LOW
//#define m_ACC			(90*m_PROPORTION)// rpm/s
#define m_ACC			(800*m_PROPORTION)// rpm/s

 #define SV_HZ        1000

//DC_motor port
#define MOTOR_PG 0 //speed
#define MOTOR_FR 1 //dir
#define MOTOR_SV 2 //speed
#define MOTOR_EN 3 //enable
#define MOTOR_BK 4 //break

//#define MOVEBASE_DEBUG
#ifdef MOVEBASE_DEBUG
#define moveprintf(x)	myprintf x
#else
#define moveprintf(x)
#endif //_MOVEBASE_DEBUG

//#define _MOVEBASE_DEBUG
#ifdef _MOVEBASE_DEBUG
#define printd(x)	myprintf x
#else
#define printd(x)
#endif //_MOVEBASE_DEBUG

//dcmotor struct
typedef struct CMotorDC_parameter_all
{
	boolean			m_bFadeMode;
	int				m_curRPM;
	int				m_targetRPM;
	unsigned long	m_timeFade;

	//enable break  dir
	Gpio_pin_parameter* m_gpioPG;//huoer
	Gpio_pin_parameter* m_gpioFR;//dir
	Gpio_pin_parameter* m_gpioSV;//speed
	Gpio_pin_parameter* m_gpioEN;//enable
	Gpio_pin_parameter* m_gpioBK;//berak

}CMotorDC_parameter;


void CMotorDC_parameter_setup(CMotorDC_parameter* CMotorDC_para);

void CMotorDC_setPin(
					 CMotorDC_parameter*	CMotorDC_para,
					 int					owner, 
					 Gpio_pin_parameter*	Gpio_pin_para
					 );
void CMotorDC_enableit( CMotorDC_parameter*	CMotorDC_para, boolean flag );
void CMotorDC_breakit( CMotorDC_parameter*	CMotorDC_para, boolean flag );
void CMotorDC_setSpeed( CMotorDC_parameter*	CMotorDC_para, long w );
void CMotorDC_setTargetSpeed( CMotorDC_parameter*	CMotorDC_para, long w );
void CMotorDC_updateSpeed( CMotorDC_parameter*	CMotorDC_para, boolean bUseExtCurRPM, int nCurRPM );
void CMotorDC_enableCoff( CMotorDC_parameter*	CMotorDC_para, int flag );
long CMotorDC_getw( CMotorDC_parameter*	CMotorDC_para );

#endif //_MotorDC_H

