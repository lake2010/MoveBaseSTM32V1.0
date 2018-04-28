#ifndef _OBD_H
#define _OBD_H
#include "DS18B20.h"
#include "config.h"

#define m_TemperNum 3

typedef struct COBD_parameter_all
{
	float	m_tmp[m_TemperNum];
	float	m_voltage;//累计电压
	float	m_stableVol;
	float	m_electricity;
	u32		m_systemTime;
	u32		m_volGetTime;
	boolean m_getTimpReady;
	int		m_volReadCount;

	DS18B20_parameter	m_DS18B20;

}CODB_parameter;

void COBD_setup(CODB_parameter* CODB_para);
void COBD_loop(CODB_parameter* CODB_para);
float COBD_getStableVol(CODB_parameter* CODB_para);

#endif
