#include "Move4.h"

CMove4::CMove4()
{
}

CMove4::~CMove4()
{
}

void CMove4::setup()
{
	m_motor[0].setPin( MOTOR_FR, m_STEPPER_DIR_PIN1, OUTPUT );
	m_motor[0].setPin( MOTOR_SV, m_STEPPER_PULSE_PIN1, OUTPUT );
	m_motor[0].setPin( MOTOR_EN, m_STEPPER_COFF_PIN1, OUTPUT );
	m_motor[0].setPin( MOTOR_BK, -1, OUTPUT );

	m_motor[1].setPin( MOTOR_FR, m_STEPPER_DIR_PIN2, OUTPUT );
	m_motor[1].setPin( MOTOR_SV, m_STEPPER_PULSE_PIN2, OUTPUT );
	m_motor[1].setPin( MOTOR_EN, m_STEPPER_COFF_PIN2, OUTPUT );
	m_motor[1].setPin( MOTOR_BK, -1, OUTPUT );
	
	m_motor[2].setPin( MOTOR_FR, m_STEPPER_DIR_PIN3, OUTPUT );
	m_motor[2].setPin( MOTOR_SV, m_STEPPER_PULSE_PIN3, OUTPUT );
	m_motor[2].setPin( MOTOR_EN, m_STEPPER_COFF_PIN3, OUTPUT );
	m_motor[2].setPin( MOTOR_BK, -1, OUTPUT );

	m_motor[3].setPin( MOTOR_FR, m_STEPPER_DIR_PIN4, OUTPUT );
	m_motor[3].setPin( MOTOR_SV, m_STEPPER_PULSE_PIN4, OUTPUT );
	m_motor[3].setPin( MOTOR_EN, m_STEPPER_COFF_PIN4, OUTPUT );
	m_motor[3].setPin( MOTOR_BK, -1, OUTPUT );
}

void CMove4::loop( int nCmdOp, const int* pParam, int nParamNum )
{
	// process cmd
	switch (nCmdOp) {
		case CMD_OP_START:
			start();
			break;
		case CMD_OP_STOP:
			stop();
			break;
		case CMD_OP_SET_V:
			if (nParamNum == 3)
				setv( pParam[0] , pParam[1], pParam[2] );
			break;
		case CMD_OP_SET_V2:
			if (nParamNum == 3)
				setv2( pParam[0] , pParam[1], pParam[2] );
			break;
		case CMD_OP_COFF_ON:
			enableCoff( 1 );
			break;
		case CMD_OP_COFF_OFF:
			enableCoff( 0 );
			break;
		default:
			break;
	}
	
	// fading speed
	for (int i = 0; i < m_MOTOR_NUM; i++) {
		m_motor[i].updateSpeed( false, 0 );
	}
}

void CMove4::start()
{
	setv( 0, 0, 0 );
	Timer1.start();
	for (int i = 0; i < m_MOTOR_NUM; i++) {
		m_motor[i].enableit( true );
		m_motor[i].breakit( false );
	}
}

void CMove4::stop()
{
	Timer1.stop();
	setv( 0, 0, 0 );
	for (int i = 0; i < m_MOTOR_NUM; i++) {
		m_motor[i].breakit( true );
	}
}

void CMove4::setv( int vx, int vy, int wz )
{
	long pw[m_MOTOR_NUM];
	moveModelBackward( vx, vy, wz, pw );
	for (int i = 0; i < m_MOTOR_NUM; i++) {
		m_motor[0].setSpeed( pw[i] );
	}
}

void CMove4::setv2( int vx, int vy, int wz )
{
	long pw[m_MOTOR_NUM];
	moveModelBackward( vx, vy, wz, pw );
	for (int i = 0; i < m_MOTOR_NUM; i++) {
		m_motor[i].setTargetSpeed( pw[i] );
	}
}

void CMove4::enableCoff( int flag )
{
	for (int i = 0; i < m_MOTOR_NUM; i++) {
		m_motor[i].enableCoff( flag );
	}
}

void CMove4::getv( int& vx, int& vy, int& wz )
{
	long pw[m_MOTOR_NUM];
	pw[0] = m_motor[0].getw();
	pw[1] = m_motor[1].getw();
	pw[2] = m_motor[2].getw();
	pw[3] = m_motor[3].getw();
	moveModelForeward( pw, vx, vy, wz );
}

const long* CMove4::getw( int& n )
{
	n = m_MOTOR_NUM;
	m_pw[0] = -m_motor[0].getw();
	m_pw[1] = m_motor[1].getw();
	m_pw[2] = -m_motor[2].getw();
	m_pw[3] = m_motor[3].getw();
	return m_pw;
}

void CMove4::moveModelForeward( const long* pw, int& vx, int& vy, int& wz )
{

}

void CMove4::moveModelBackward( int vx, int vy, int wz, long* pw )
{

}

