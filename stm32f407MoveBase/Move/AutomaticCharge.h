// AutomaticCharge.h

#ifndef _AUTOMATICCHARGE_h
#define _AUTOMATICCHARGE_h

#include "config.h"
#include "IRreceive.h"

#define AC_MOVEBASE_DEBUG
#ifdef AC_MOVEBASE_DEBUG
#define printddd(x)	myprintf x
#else
#define printddd(x)
#endif //_MOVEBASE_DEBUG

#define m_INFRARED_RECIEVED	0x426F6F63	//Booc
#define m_receiveIRdetTime	1500		//时间间隔内没有收到IR信号
#define m_chargeDetTime		1800000	//半小时检测一次
#define m_ChargeUsingTim	50000	//允许对接用时最长时间
#define m_WaitShutDownTime	60000	//等待上位关机时间
#define m_testIRsignal      2000

#define IR_TIMER				TIM4
typedef enum {
	charge_IDEL = 0,
	charge_testIRsignal,
	charge_gotoCharge,
	charge_isReady,
	charge_testCharge,
	charge_gotoWork,
	charge_waitShutdown,
} charge_state;

#define m_AC_CHARGE				1

typedef struct CAutomaticCharge_parameter_all
{

	IRSendRev_parameter myIR[2];
	decode_results_parameter result;
	//重新规划-------------------
	u8   m_chargeState;
	u8   m_isChargeing;
	u8	 m_testIRsignalcnt;
	u32  m_sytemTime;
	u32  m_chargeUsingTime;
}CAutoCharge_parameter;


void CAutoCharge_setup(CAutoCharge_parameter* CAutoCharge_para);
void CAutoCharge_loop(CAutoCharge_parameter* CAutoCharge_para,
					  int *nCmdOp, const int *pParam, int nParamNum);
void CAutoCharge_start(CAutoCharge_parameter* CAutoCharge_para);
void CAutoCharge_stop(CAutoCharge_parameter* CAutoCharge_para);//
void CAutoCharge_interrupt(CAutoCharge_parameter* CAutoCharge_para);
int  CAutoCharge_isGotoWork(CAutoCharge_parameter* CAutoCharge_para);
int  CAutoCharge_isChargeing(CAutoCharge_parameter* CAutoCharge_para);
#endif
