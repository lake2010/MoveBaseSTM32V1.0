#ifndef _Move2_H
#define _Move2_H

// TODO: change your motor
#include "MotorDC.h"
//#include "MotorStepper.h"

#define m_MOTOR_NUM		2
#define m_MOTOR_MAX_PW	20*314//单个轮子最大角速度

typedef struct CMove2_parameter_all
{
	// TODO: change your motor
	CMotorDC_parameter m_motor[m_MOTOR_NUM]; // 0 is left  1 is right
	//CStepper m_motor[m_MOTOR_NUM];
	long m_pw[m_MOTOR_NUM];
}CMove2_parameter;

void CMove2_setup( CMove2_parameter* CMove2_para );
void CMove2_loop( CMove2_parameter* CMove2_para, int nCmdOp, const int* pParam, int nParamNum );
void CMove2_start( CMove2_parameter* CMove2_para );
void CMove2_stop( CMove2_parameter* CMove2_para );
void CMove2_setv( CMove2_parameter* CMove2_para, int vx, int vy, int wz );
void CMove2_setv2( CMove2_parameter* CMove2_para, int vx, int vy, int wz );
void CMove2_enableCoff( CMove2_parameter* CMove2_para, int flag );
void CMove2_getv( CMove2_parameter* CMove2_para, int* vx, int* vy, int* wz );
const long* CMove2_getw( CMove2_parameter* CMove2_para, int* n );

#endif //_Move2_H
