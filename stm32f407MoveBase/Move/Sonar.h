#ifndef _Sonar_H
#define _Sonar_H

#include "config.h"
#include "I2C.h"

//#define SONAR_DEBUG
#ifdef SONAR_DEBUG
#define sonarprintf(x)	myprintf x
#else
#define sonarprintf(x)
#endif //_MOVEBASE_DEBUG

#define m_SONAR_NUM		3
#define m_PING_INTERVAL	40 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define m_MAX_DIST		200 // cm
#define m_WARNING_DIST	12 // cm
#define m_STOP_DIST		4 // cm
#define m_REG			2//距离寄存器地址
#define m_INT			0x75//0x72;//0x72降噪等级3,0x75降噪等级6
#define m_SONAR_MODE	0xb2//0xb4;//距离滤波等级
#define m_SONAR_TEMP	0Xc9

typedef struct CSonar_parameter_all
{
	int m_SONAR_KS103_ADDRESS[m_SONAR_NUM];
	unsigned int m_cm[m_SONAR_NUM]; // Where the ping distances are stored.
	int m_currentSensor;            // Keeps track of which sensor is active.	
	unsigned long m_pingTimer[m_SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
	boolean m_bReady;
	int m_event;
	int m_warning;
	unsigned long m_wait_time;
	unsigned long m_detTimer;
	boolean m_fEventProcessed;
	unsigned int m_minDistance;
	boolean m_vcontrol;
	boolean m_lastvcontrol;
	boolean m_start_tmp;
	unsigned long m_start_temp;
	unsigned long m_det_temp;
	double m_sonar_temp;
}CSonar_parameter;

void CSonar_setup( CSonar_parameter* CSonar_para );
void CSonar_loop( CSonar_parameter* CSonar_para, int nCmdOp, const int* pParam, int nParamNum );
int CSonar_getEvent( CSonar_parameter* CSonar_para );
int CSonar_getWarning( CSonar_parameter* CSonar_para, unsigned int *m_minDis );

#endif //_Sonar_H
