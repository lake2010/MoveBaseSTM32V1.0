#ifndef _cmd_H
#define _cmd_H


// TODO: use your com
#include "SerialCom.h"

typedef struct CCmd_param_all
{
	// TODO: use your com
	CSerialCom_param m_com;
}CCmd_param;

void CCmd_setup( CCmd_param* CCmd_para );
CMD_STRING* CCmd_recvCmd( CCmd_param* CCmd_par, int* nCmdNum );
int CCmd_parseCmd( CCmd_param* CCmd_par, const char* strCmd, int* pParam, int* nParamNum );

#endif //_cmd_H

