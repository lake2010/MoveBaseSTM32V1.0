#ifndef _MoveBase_H
#define _MoveBase_H

// TODO:
#include "cmd.h"
//#include "MyCmd.h"

#include "Move2.h"
//#include "Move4.h"

//#include "VirtualODO.h"
#include "ODO2PG.h"
//#include "ODO2STEPPER.h"

#include "Sonar.h"
#include "OBD.h"
//#include "Temper.h"
#include "AutomaticCharge.h"
#include "cmyserial.h"
// TODO END

#include "LED.h"
#include "CAN.h"

//#define _MOVEBASE_DEBUG
//#ifdef _MOVEBASE_DEBUG
//#define printd(x)	myprintf x
//#else
//#define printd(x)
//#endif //_MOVEBASE_DEBUG

//#define MOVEBASE_DEBUG
//#ifdef MOVEBASE_DEBUG
//#define moveprintf(x)	myprintf x
//#else
//#define moveprintf(x)
//#endif //_MOVEBASE_DEBUG
typedef struct SYSstatus_parameter_all
{
	char  charge_status; // .
	char  driver_status;
	char  IO_status;
	char  Sys_status;
  unsigned long m_SYStimeLastRead ;// 
}SYSstatus_parameter;


typedef struct CMoveBase_parameter_all
{
	CMySerial_parameter		m_ccm; //串口4接收发送指令
	CMySerial_parameter		m_ccm3;//串口3接收发送指令
	CCAN_parameter				m_canccm;
	CCmd_param				m_cmd;//message 老协议形式
	CMove2_parameter		m_move;//two motor
	CODO2PG_parameter		m_odo;//odo message
	CAutoCharge_parameter	m_ac;
	CSonar_parameter		m_sonar;
	CLED_parameter			m_LED;
	CODB_parameter			m_obd;
	SYSstatus_parameter 	m_SYSstatus;  /*   系统状态  */
	unsigned long			m_heartBeat;
	unsigned long			m_accTimer;
	int						IRstate;
	int						softstop_state;
	int 					getcmd_state;
	int 					getCoff_state;//获取上位机充电下发的充电指令状态
	int						softLocking_state;
	int 					getStartUpCmdTime;
	uint16_t 			timeToOn;
	uint32_t			timeToOff;
}CMoveBase_parameter;



void SYSstatus_loop( SYSstatus_parameter* SYSstatus_para);
void SYSstatus_setup( SYSstatus_parameter* SYSstatus_para);

extern CMoveBase_parameter MoveBase;
void CMoveBase_setup( CMoveBase_parameter* CMoveBase_para );
void CMoveBase_loop( CMoveBase_parameter* CMoveBase_para );

void CMoveBase_softStop(CMoveBase_parameter* CMoveBase_para);
void CMoveBase_softLocking(CMoveBase_parameter* CMoveBase_para);

/*
// TODO:	
//CVirtualODO m_odo;

//ODO2STEPPER m_odo;
	
;

CTemper m_temper;

CAutomaticCharge m_ac;
// TODO END
*/

#endif //_MoveBase_H
