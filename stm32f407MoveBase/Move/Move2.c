#include "commen.h"
#include "Move2.h"
#include "cmd.h"

void CMove2_setup( CMove2_parameter* CMove2_para )
{
	//motor setup
	for( int i = 0; i < m_MOTOR_NUM; i++ )
		CMotorDC_parameter_setup( &CMove2_para->m_motor[i] );
	//left
	CMotorDC_setPin( &CMove2_para->m_motor[0], 
					 MOTOR_FR, 
					 &m_FR_Port_L);
	CMotorDC_setPin( &CMove2_para->m_motor[0], 
					 MOTOR_SV, 
					 &m_SV_Port_L );
	CMotorDC_setPin( &CMove2_para->m_motor[0], 
					 MOTOR_EN, 
					 &m_EN_Port_L );
	CMotorDC_setPin( &CMove2_para->m_motor[0], 
					 MOTOR_BK, 
					 &m_BK_Port_L );

	//right
	CMotorDC_setPin( &CMove2_para->m_motor[1], 
					 MOTOR_FR, 
					 &m_FR_Port_R );
	CMotorDC_setPin( &CMove2_para->m_motor[1], 
					 MOTOR_SV, 
					 &m_SV_Port_R );
	CMotorDC_setPin( &CMove2_para->m_motor[1], 
					 MOTOR_EN, 
					 &m_EN_Port_R );
	CMotorDC_setPin( &CMove2_para->m_motor[1], 
					 MOTOR_BK, 
					 &m_BK_Port_R );
}

void CMove2_loop( CMove2_parameter* CMove2_para, int nCmdOp, const int* pParam, int nParamNum )
{
	// process cmd
	switch (nCmdOp) {
		case CMD_OP_START:
			CMove2_start( CMove2_para );
			break;
		case CMD_OP_STOP:
			CMove2_stop( CMove2_para );
			break;
		case CMD_OP_SET_V:
			if (nParamNum == 3)
				CMove2_setv( CMove2_para, pParam[0] , pParam[1], pParam[2] );
			break;
		case CMD_OP_SET_V2:
			if (nParamNum == 3)
				CMove2_setv2( CMove2_para, pParam[0] , pParam[1], pParam[2] );
			break;
		case CMD_OP_COFF_ON:
			CMove2_enableCoff( CMove2_para, 1 );
			break;
		case CMD_OP_COFF_OFF:
			CMove2_enableCoff( CMove2_para, 0 );
			break;
		default:
			break;
	}
	
	// fading speed
	for (int i = 0; i < m_MOTOR_NUM; i++) {
		CMotorDC_updateSpeed( &CMove2_para->m_motor[i], false, 0 );
	}
}

void CMove2_start( CMove2_parameter* CMove2_para )
{
//	TIM_Cmd(TIM5, ENABLE);
//	TIM_Cmd(TIM4, ENABLE);
	//Œª÷√ªª¡À
	CMove2_setv( CMove2_para, 0, 0, 0 );
	for (int i = 0; i < m_MOTOR_NUM; i++) {
		CMotorDC_enableit( &CMove2_para->m_motor[i], true );
		CMotorDC_breakit( &CMove2_para->m_motor[i], false );
	}
	
}

void CMove2_stop( CMove2_parameter* CMove2_para )
{
	CMove2_setv( CMove2_para, 0, 0, 0 );
	TIM_Cmd(TIM5, DISABLE);
	TIM_Cmd(TIM4, DISABLE);
	for (int i = 0; i < m_MOTOR_NUM; i++) {
		CMotorDC_breakit( &CMove2_para->m_motor[i], true );
	}
}

void CMove2_moveModelBackward( CMove2_parameter* CMove2_para, int vx, int vy, int wz, long* pw )
{
	double pv[2];
	pv[0] = (int)( (double)vx - ((double)wz * (double)m_WHEEL_D) / (double)2000 );
	pv[1] = (int)( (double)vx + ((double)wz * (double)m_WHEEL_D) / (double)2000 );
	for (int i = 0; i < 2; i++) {
		pw[i] = (long)((pv[i] * (double)1000) / (double)m_WHEEL_R);
	}

	double pw_MAX_ABS = ABS(pw[0]);
	double pw1ABS = ABS(pw[1]);
	pw_MAX_ABS = MAX(pw_MAX_ABS, pw1ABS);
	if( pw_MAX_ABS > m_MOTOR_MAX_PW ){
		double tmp = m_MOTOR_MAX_PW / pw_MAX_ABS;
		pw[0] = (long)( (double)pw[0] * tmp );
		pw[1] = (long)( (double)pw[1] * tmp );
	}

	moveprintf(( "pw[0]:%ld pw[1]:%ld\n", pw[0], pw[1] ));
	moveprintf(( "vx:%d vy:%d wz:%d\n", vx, vy, wz ));	
}

void CMove2_moveModelForeward( CMove2_parameter* CMove2_para, const long* pw, int* vx, int* vy, int* wz )
{
	double pv[m_MOTOR_NUM];
	for (int i = 0; i < m_MOTOR_NUM; i++) {
		pv[i] = ((double)pw[i] * (double)m_WHEEL_R) / (double)1000;
	}
	*vx = (int)((pv[0] + pv[1]) / 2.0);
	*vy = 0;
	*wz = (int)(((pv[1] - pv[0]) * (double)1000) / (double)m_WHEEL_D);
	moveprintf(( "vx:%d vy:%d wz:%d\n", *vx, *vy, *wz ));
}

void CMove2_setv( CMove2_parameter* CMove2_para, int vx, int vy, int wz )
{
	long pw[m_MOTOR_NUM];
	CMove2_moveModelBackward( CMove2_para, vx, vy, wz, pw );
	CMotorDC_setSpeed( &CMove2_para->m_motor[0], -pw[0] );
	CMotorDC_setSpeed( &CMove2_para->m_motor[1], pw[1] );
    printd(( "-p0=%ld, p1=%ld\n", -pw[0], pw[1] ));
}

void CMove2_setv2( CMove2_parameter* CMove2_para, int vx, int vy, int wz )
{
	long pw[m_MOTOR_NUM];
	CMove2_moveModelBackward( CMove2_para, vx, vy, wz, pw );
	CMotorDC_setTargetSpeed( &CMove2_para->m_motor[0], -pw[0] );
	CMotorDC_setTargetSpeed( &CMove2_para->m_motor[1], pw[1] );
}

void CMove2_enableCoff( CMove2_parameter* CMove2_para, int flag )
{
	for (int i = 0; i < m_MOTOR_NUM; i++) {
		CMotorDC_enableCoff( &CMove2_para->m_motor[i], flag );
	}
}

void CMove2_getv( CMove2_parameter* CMove2_para, int* vx, int* vy, int* wz )
{
	long pw[m_MOTOR_NUM];
	pw[0] = -CMotorDC_getw( &CMove2_para->m_motor[0] );
	pw[1] = CMotorDC_getw( &CMove2_para->m_motor[1] );
	CMove2_moveModelForeward( CMove2_para, pw, vx, vy, wz );
}

const long* CMove2_getw( CMove2_parameter* CMove2_para, int* n )
{
	*n = m_MOTOR_NUM;
	CMove2_para->m_pw[0] = -CMotorDC_getw( &CMove2_para->m_motor[0] );
	CMove2_para->m_pw[1] = CMotorDC_getw( &CMove2_para->m_motor[1] );
	return CMove2_para->m_pw;
}
