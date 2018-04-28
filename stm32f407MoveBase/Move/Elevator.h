#ifndef _ELEVATOR_H
#define _ELEVATOR_H

#include "config.h"
#include "ICmd.h"
#define ACTION		true
#define LIMITACTION	false
typedef enum CElevatorState_all
{
	UP = 0,
	UPDIS,
	DOWN,
	DOWNDIS,
	STOP,
	IDLE
}CElevatorState;

typedef enum CElevatorLimitState_all
{
	UPLIMIT = 0,
	DOWNLIMIT,
	NOLIMIT
}CElevatorLimitState;

typedef enum CElevatorTestState_all
{
	GODOWN,
	GOUP
}CElevatorTestState;

typedef struct CElevator_parameter_all
{
	u32					m_goTime;
	u32					m_startTime;
	Gpio_pin_parameter*	m_upPin;
	Gpio_pin_parameter*	m_downPin;
	Gpio_pin_parameter*	m_downLimitPin;
	Gpio_pin_parameter*	m_upLimitPin;
	CElevatorState		m_ElevatorState;
	CElevatorLimitState m_ElevatorLimitState;
	CElevatorTestState	m_ElevatorTestState;

}CElevator_parameter;

#define ALLWAYTIME	1000
#define ALLWAYLONG	1000 //mm
void CElevator_setup( CElevator_parameter* CElevator_para);
void CElevator_loop( CElevator_parameter* CElevator_para,
					 int nCmdOp,
					 const int* pParam,
					 int nParamNum );
void CElevator_up(CElevator_parameter* CElevator_para);
void CElevator_down(CElevator_parameter* CElevator_para);
void CElevator_stop(CElevator_parameter* CElevator_para);
void CElevator_test(CElevator_parameter* CElevator_para);

#endif // !_ELEVATOR_H
