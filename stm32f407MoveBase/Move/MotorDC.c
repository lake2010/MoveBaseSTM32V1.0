#include "MotorDC.h"
#include "config.h"
#include "MoveBase.h"

void CMotorDC_setMotorSpeed( CMotorDC_parameter* CMotorDC_para, int rpm );
long CMotorDC_rpm2w( CMotorDC_parameter* CMotorDC_para, int rpm );
void CMotorDC_setDir( CMotorDC_parameter* CMotorDC_para, int dir );
void CMotorDC_setPWM( CMotorDC_parameter* CMotorDC_para, int rpm );

void CMotorDC_parameter_setup(CMotorDC_parameter* CMotorDC_parameter)
{
	CMotorDC_parameter->m_bFadeMode = false;
	CMotorDC_parameter->m_curRPM = 0;
}

//enable break  dir OUTPUT, speedpin define but do not init
void CMotorDC_setPin( CMotorDC_parameter*	CMotorDC_para,
					  int					owner, 
					  Gpio_pin_parameter*	Gpio_pin_para)
{
	switch (owner) {
		case MOTOR_PG:
			CMotorDC_para->m_gpioPG	= Gpio_pin_para;
			break;
		case MOTOR_FR:
			CMotorDC_para->m_gpioFR	= Gpio_pin_para;
			pinOutputModeInit( Gpio_pin_para );
			break;
		case MOTOR_SV:
			CMotorDC_para->m_gpioSV	= Gpio_pin_para;
			break;
		case MOTOR_EN:
			CMotorDC_para->m_gpioEN	= Gpio_pin_para;
			pinOutputModeInit( Gpio_pin_para );
			break;
		case MOTOR_BK:
			CMotorDC_para->m_gpioBK	= Gpio_pin_para;
			pinOutputModeInit( Gpio_pin_para );
			break;
		default:
			break;
	}
}

int CMotorDC_w2rpm( CMotorDC_parameter* CMotorDC_para, long w )
{
	double rpm = ( (double)w * ( ( (double)30 * (double)PI_DEN / (double)1000 ) * (double)m_PROPORTION ) ) / (double)PI_NUM;
	LIMIT_RANGE( rpm, -m_MAX_RPM, m_MAX_RPM );
	moveprintf(( "rpm:%d\n",(int)rpm ));
	return (int)rpm;
}

void CMotorDC_enableit( CMotorDC_parameter* CMotorDC_para, boolean flag )
{
	if (flag)
		pinDigitalWrite( CMotorDC_para->m_gpioEN, m_ENABLE );
	else
		pinDigitalWrite( CMotorDC_para->m_gpioEN, ! m_ENABLE );
}

void CMotorDC_breakit( CMotorDC_parameter* CMotorDC_para, boolean flag )
{
	if (flag)
		pinDigitalWrite( CMotorDC_para->m_gpioBK, m_BREAK );
	else
		pinDigitalWrite( CMotorDC_para->m_gpioBK, ! m_BREAK );
}

void CMotorDC_setSpeed( CMotorDC_parameter* CMotorDC_para, long w )
{
	int rpm = CMotorDC_w2rpm( CMotorDC_para, w );
	CMotorDC_setMotorSpeed( CMotorDC_para, rpm );
	CMotorDC_para->m_bFadeMode = false;
	CMotorDC_para->m_curRPM = rpm;
	moveprintf(( "rpm=%d\n",rpm ));
}

void CMotorDC_setTargetSpeed( CMotorDC_parameter* CMotorDC_para, long w )
{
	CMotorDC_para->m_targetRPM = CMotorDC_w2rpm( CMotorDC_para, w );
	CMotorDC_para->m_bFadeMode = true;
	CMotorDC_para->m_timeFade = millis();
}

void CMotorDC_updateSpeed( CMotorDC_parameter* CMotorDC_para, boolean bUseExtCurRPM, int nCurRPM )
{
	if (! CMotorDC_para->m_bFadeMode)
		return;
	
	unsigned long dt = millis() - CMotorDC_para->m_timeFade;
	if (dt >= 100) {
		CMotorDC_para->m_timeFade = millis();
		if (dt < 200) {
			if (bUseExtCurRPM)
				CMotorDC_para->m_curRPM = nCurRPM;
			
			if (CMotorDC_para->m_curRPM < CMotorDC_para->m_targetRPM) {
				CMotorDC_para->m_curRPM += m_ACC * dt / 1000;
				LIMIT_MAX( CMotorDC_para->m_curRPM, CMotorDC_para->m_targetRPM );
			} else if (CMotorDC_para->m_curRPM > CMotorDC_para->m_targetRPM) {
				CMotorDC_para->m_curRPM -= m_ACC * dt / 1000;
				LIMIT_MIN( CMotorDC_para->m_curRPM, CMotorDC_para->m_targetRPM );
			} else {
				return;
			}
			CMotorDC_setMotorSpeed( CMotorDC_para, CMotorDC_para->m_curRPM );
		}
	}
}

long CMotorDC_getw(CMotorDC_parameter* CMotorDC_para)
{
	return CMotorDC_rpm2w( CMotorDC_para, CMotorDC_para->m_curRPM );
}

long CMotorDC_rpm2w( CMotorDC_parameter* CMotorDC_para, int rpm )
{
	double w = ((double)rpm * (double)PI_NUM) / (((double)30 * (double)PI_DEN / (double)1000) * (double)m_PROPORTION);
	return (long)w;
}

void CMotorDC_setMotorSpeed( CMotorDC_parameter* CMotorDC_para, int rpm )
{
	if (rpm < 0) {
		CMotorDC_setDir( CMotorDC_para, !m_FOREWARD );
		printd(( "dir=%d\n", !m_FOREWARD ));
		rpm = - rpm;
	} else {
		CMotorDC_setDir( CMotorDC_para, m_FOREWARD );
		printd(( "dir=%d\n", m_FOREWARD ));
	}
	CMotorDC_setPWM( CMotorDC_para, rpm );
}

void CMotorDC_setDir( CMotorDC_parameter* CMotorDC_para, int dir )
{
	pinDigitalWrite( CMotorDC_para->m_gpioFR, dir );
}

void CMotorDC_setPWM( CMotorDC_parameter* CMotorDC_para, int rpm )
{
	//  中大电机，以PWM波占空比控制电机转速   变占空比，固定频率//
		static int breakFlag; 
		float frequency_d = 0;
		frequency_d = m_MAX_FREQUWNCY /SV_HZ;
		unsigned int frequency = (unsigned int)(frequency_d);
		double duty_d = ( (double)rpm * (double)frequency ) / (double)m_MAX_RPM;//m_MAX_FREQUWNCY
		unsigned int duty = (unsigned int)(duty_d);
		//抱死开关
		if(MoveBase.softLocking_state)
		{
			breakFlag = 2; //2个电机
			if(rpm == 0)
				CMotorDC_breakit(CMotorDC_para,true);	
			else
				CMotorDC_breakit(CMotorDC_para,false);
		}
		else if(breakFlag)
		{
				breakFlag--;
				CMotorDC_breakit(	CMotorDC_para, false );
		}
		if (rpm == 0){
		if( (CMotorDC_para->m_gpioSV) == &m_SV_Port_L )
		{
			TIM_Cmd(TIM5, DISABLE);
		}
		else if( (CMotorDC_para->m_gpioSV) == &m_SV_Port_R )
		{
			TIM_Cmd(TIM4, DISABLE);
		}
		CMotorDC_enableit( CMotorDC_para, false );
		return;
	}

	if( (CMotorDC_para->m_gpioSV) == &m_SV_Port_L ){
		TIM_SetCompare3(TIM5,duty);//左轮由TIM5控制,选择第三通道
		TIM_SetAutoreload(TIM5,frequency);
	}else if( (CMotorDC_para->m_gpioSV) == &m_SV_Port_R ){//右轮由TIM4控制
		TIM_SetCompare1(TIM4,duty);//选择第1通道
		TIM_SetAutoreload(TIM4,frequency);
	}

	if( rpm != 0 ){
		CMotorDC_enableit( CMotorDC_para, true );
		if( (CMotorDC_para->m_gpioSV) == &m_SV_Port_L )
		TIM_Cmd(TIM5, ENABLE);
		else if( (CMotorDC_para->m_gpioSV) == &m_SV_Port_R )
		TIM_Cmd(TIM4, ENABLE);
	}
}

void CMotorDC_enableCoff( CMotorDC_parameter*	CMotorDC_para, int flag )
{
	
}
