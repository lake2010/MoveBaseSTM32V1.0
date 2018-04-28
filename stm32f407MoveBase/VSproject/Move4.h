#ifndef _Move4_H
#define _Move4_H

// TODO: change your motor
#include "MotorDC.h"

class CMove4 : public IMove
{
public:
	CMove4();
	virtual ~CMove4();
	virtual void setup();
	virtual void loop( int nCmdOp, const int* pParam, int nParamNum );
	virtual void start();
	virtual void stop();
	virtual void setv( int vx, int vy, int wz );
	virtual void setv2( int vx, int vy, int wz );
	virtual void enableCoff( int flag );
	virtual void getv( int& vx, int& vy, int& wz );
	virtual const long* getw( int& n );

private:
	static const int m_STEPPER_PULSE_PIN1 =  22;
	static const int m_STEPPER_DIR_PIN1 =    23;
	static const int m_STEPPER_COFF_PIN1 =   24;
	static const int m_STEPPER_PULSE_PIN2 =  30; 
	static const int m_STEPPER_DIR_PIN2 =    31;
	static const int m_STEPPER_COFF_PIN2 =   32;
	static const int m_STEPPER_PULSE_PIN3 =  26; 
	static const int m_STEPPER_DIR_PIN3 =    27;
	static const int m_STEPPER_COFF_PIN3 =   28;
	static const int m_STEPPER_PULSE_PIN4 =  34; 
	static const int m_STEPPER_DIR_PIN4 =    35;
	static const int m_STEPPER_COFF_PIN4 =   36;
	
	/*
	static const int m_WHEEL_R =  152; // mm
	static const int m_VICHLE_S = 392; // mm
	static const int m_VICHLE_D = 420; // mm
	*/
	static const int m_MOTOR_NUM = 4;

	// TODO: change your motor
	CMotorDC m_motor[m_MOTOR_NUM]; // 0 is left，1 is right
	
	long m_pw[m_MOTOR_NUM];
	
	void moveModelForeward( const long* pw, int& vx, int& vy, int& wz );
	void moveModelBackward( int vx, int vy, int wz, long* pw );
};

#endif //_Move4_H

