#ifndef _ODO2PG_H
#define _ODO2PG_H
#include "config.h"
#include "Move2.h"
#define I2C_SONAR
//#define m_MOTOR_NUM 2

#define m_DEC_NUM		30
#define m_PROPORTION	50
#define m_WHEEL_DEC_NUM	(m_DEC_NUM * m_PROPORTION)
#define m_MAG_PRO		5 * m_PROPORTION

#define m_Ts			100 // ms

typedef struct CODO2PG_parameter_all
{
	const long*		m_pwFrMove;
	int				m_counter_l;
	int				m_counter_r;
	unsigned long	m_timeLastRead;
	long			m_pw[m_MOTOR_NUM];
	int				m_vx, m_vy, m_wz;
	unsigned long	m_start_time;
	unsigned long	m_odo_count;
}CODO2PG_parameter;


void CODO2PG_setup( CODO2PG_parameter* CODO2PG_para );
void CODO2PG_loop( CODO2PG_parameter* CODO2PG_para);
void CODO2PG_getv( CODO2PG_parameter* CODO2PG_para, int* vx, int* vy, int* wz ); // vx, vy: mm/s; wz: rad/1000/s.
const long* CODO2PG_getw( CODO2PG_parameter* CODO2PG_para, int* n ); // return w: rad/1000/s

void CODO2PG_pg_l_int( CODO2PG_parameter* CODO2PG_para );
void CODO2PG_pg_r_int( CODO2PG_parameter* CODO2PG_para );

void CODO2PG_couter2rad( CODO2PG_parameter* CODO2PG_para, const long* pCounter, int dt, long* pw );
void CODO2PG_moveModelForeward( CODO2PG_parameter* CODO2PG_para, const long* pw, int* vx, int* vy, int* wz );


#endif //_ODO2PG_H

