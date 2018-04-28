#include "Elevator.h"

CElevatorLimitState_all CElevator_checkLimit( CElevator_parameter* CElevator_para );

void CElevator_setup( CElevator_parameter* CElevator_para )
{
	CElevator_para->m_downPin		=	&m_Elevator_DOWN;
	CElevator_para->m_upPin			=	&m_Elevator_UP;
	CElevator_para->m_upLimitPin	=	&m_Elevator_UP_Limit;
	CElevator_para->m_downLimitPin	=	&m_Elevator_DOWN_Limit;
	CElevator_para->m_ElevatorState = IDLE;
	pinOutputModeInit(CElevator_para->m_upPin);
	pinOutputModeInit(CElevator_para->m_downPin);
	pinInputModeInit(CElevator_para->m_upLimitPin);
	pinInputModeInit(CElevator_para->m_downLimitPin);
	CElevator_stop(CElevator_para);
	CElevator_down(CElevator_para);
}

void CElevator_loop( CElevator_parameter* CElevator_para,
					int nCmdOp,
					const int* pParam,
					int nParamNum )
{
	switch (nCmdOp) {
		case CMD_OP_ELEVATORUP:
			if( CElevator_para->m_ElevatorState != UP ||
				CElevator_para->m_ElevatorState != UPDIS){
					if( nParamNum == 0 )
						CElevator_para->m_ElevatorState = UP;
					else if( nParamNum == 1 ){
						CElevator_para->m_ElevatorState = UPDIS;					
						CElevator_para->m_goTime = (float)pParam[0] / (float)ALLWAYLONG * 
												   (float)ALLWAYTIME;
						CElevator_para->m_startTime = millis();
					}
					else{
						CElevator_stop(CElevator_para);
						CElevator_para->m_ElevatorState = IDLE;
						break;
					}
					CElevator_up(CElevator_para);
			}
			break;
		case CMD_OP_ELEVATORDOWN:
			if( CElevator_para->m_ElevatorState != DOWN ||
				CElevator_para->m_ElevatorState != DOWNDIS){
					if( nParamNum == 0 )
						CElevator_para->m_ElevatorState = DOWN;
					else if( nParamNum == 1 ){
						CElevator_para->m_ElevatorState = DOWNDIS;
						CElevator_para->m_goTime = (float)pParam[0] / (float)ALLWAYLONG * 
												   (float)ALLWAYTIME;
						CElevator_para->m_startTime = millis();
					}
					else{
						CElevator_stop(CElevator_para);
						CElevator_para->m_ElevatorState = IDLE;
						break;
					}
					CElevator_down(CElevator_para);
			}
			break;
		case CMD_OP_ELEVATORTEST:
			break;
		case CMD_OP_ELEVATORSTOP:
			CElevator_para->m_ElevatorState = STOP;
			CElevator_stop(CElevator_para);
			break;
		default:
			break;
	}

	if( CElevator_para->m_ElevatorState == UP || 
		CElevator_para->m_ElevatorState == UPDIS ){
			if( CElevator_checkLimit(CElevator_para) == UPLIMIT ){//到达 上限位
				CElevator_stop(CElevator_para);
				CElevator_para->m_ElevatorState = IDLE;
				myprintf("@uplimit#");
			}else if( CElevator_para->m_ElevatorState == UPDIS ){
				if( millis() - CElevator_para->m_startTime >= CElevator_para->m_goTime ){
					CElevator_stop(CElevator_para);
					CElevator_para->m_ElevatorState = IDLE;
					myprintf("@updisover#");
				}
			}
	}else if( CElevator_para->m_ElevatorState == DOWN || 
			  CElevator_para->m_ElevatorState == DOWNDIS ){
				if( CElevator_checkLimit(CElevator_para) == DOWNLIMIT ){//到达 上限位
					CElevator_stop(CElevator_para);
					CElevator_para->m_ElevatorState = IDLE;
					myprintf("@downlimit#");
				}else if( CElevator_para->m_ElevatorState == DOWNDIS ){
					if( millis() - CElevator_para->m_startTime >= CElevator_para->m_goTime ){
						CElevator_stop(CElevator_para);
						CElevator_para->m_ElevatorState = IDLE;
						myprintf("@downdisover#");
					}
				}
	}else if( CElevator_para->m_ElevatorState == STOP ){
		CElevator_stop(CElevator_para);
		CElevator_para->m_ElevatorState = STOP;
	}else if( CElevator_para->m_ElevatorState == IDLE ){

	}
}

void CElevator_up(CElevator_parameter* CElevator_para)
{
	//CElevator_para->m_ElevatorState = UP;
	CElevator_stop(CElevator_para);
	pinDigitalWrite( CElevator_para->m_upPin, ACTION );
	pinDigitalWrite( CElevator_para->m_downPin, !ACTION );
}

void CElevator_down(CElevator_parameter* CElevator_para)
{
	//CElevator_para->m_ElevatorState = DOWN;
	CElevator_stop(CElevator_para);
	pinDigitalWrite( CElevator_para->m_upPin, !ACTION );
	pinDigitalWrite( CElevator_para->m_downPin, ACTION );
}

void CElevator_stop(CElevator_parameter* CElevator_para)
{
	//CElevator_para->m_ElevatorState = STOP;
	CElevator_stop(CElevator_para);
	pinDigitalWrite( CElevator_para->m_upPin, !ACTION );
	pinDigitalWrite( CElevator_para->m_downPin, !ACTION );
}

CElevatorLimitState_all CElevator_checkLimit( CElevator_parameter* CElevator_para )
{
	if( pinDigitalRead( CElevator_para->m_upLimitPin ) == LIMITACTION ){
		CElevator_para->m_ElevatorLimitState = UPLIMIT;
		CElevator_stop(CElevator_para);
		myprintf("@uplimit#");
		CElevator_para->m_ElevatorState = IDLE;
		return CElevator_para->m_ElevatorLimitState;
	}else if( pinDigitalRead( CElevator_para->m_downLimitPin ) == LIMITACTION ){
		CElevator_para->m_ElevatorLimitState = DOWNLIMIT;
		CElevator_stop(CElevator_para);
		myprintf("@downlimit#");
		CElevator_para->m_ElevatorState = IDLE;
		return CElevator_para->m_ElevatorLimitState;
	}else{
		CElevator_para->m_ElevatorLimitState = NOLIMIT;
		return CElevator_para->m_ElevatorLimitState;
	}
}

void CElevator_test(CElevator_parameter* CElevator_para)
{

}