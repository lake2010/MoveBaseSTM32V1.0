#include "cmd.h"
#include "config.h"
#include "stdlib.h"

void CCmd_setup( CCmd_param* CCmd_para )
{
	CCmd_para->m_com.m_nCmdNum = 0;
	CCmd_para->m_com.m_SerialBuf.m_nDataLen = 0;
}

CMD_STRING* CCmd_recvCmd( CCmd_param* CCmd_par, int* nCmdNum )
{
	CSerialCom_recv(&CCmd_par->m_com);
	return CSerialCom_getCmd( &CCmd_par->m_com, nCmdNum );
}

int CCmd_parseCmd( CCmd_param* CCmd_par, const char* strCmd, int* pParam, int* nParamNum )
{
//	char strTmp[MAX_CMD_STRLEN];
//	strcpy( strTmp, strCmd + 1 );
//	strTmp[strlen(strTmp) - 1] = 0;

//	int op = -1;
//	*nParamNum = 0;
//	
//	char* p = strtok( strTmp, " " );
//	if (0 == strcmp( p, CMD_STR_START )) // case sensitive
//		op = CMD_OP_START;
//	else if (0 == strcmp( p, CMD_STR_STOP ))
//		op = CMD_OP_STOP;
//	else if (0 == strcmp( p, CMD_STR_SET_V ))
//		op = CMD_OP_SET_V;
//	else if (0 == strcmp( p, CMD_STR_SET_V2 ))
//		op = CMD_OP_SET_V2;
//	else if (0 == strcmp( p, CMD_STR_DEC ))
//		op = CMD_OP_DEC;
//	else if (0 == strcmp( p, CMD_STR_GOTOX ))
//		op = CMD_OP_GOTOX;
//	else if (0 == strcmp( p, CMD_STR_GOTOY ))
//		op = CMD_OP_GOTOY;
//	else if (0 == strcmp( p, CMD_STR_ROTATE ))
//		op = CMD_OP_ROTATE;
//	else if (0 == strcmp( p, CMD_STR_COFF_ON ))
//		op = CMD_OP_COFF_ON;
//	else if (0 == strcmp( p, CMD_STR_COFF_OFF ))
//		op = CMD_OP_COFF_OFF;
//	else if (0 == strcmp( p, CMD_STR_AC_ON ))
//		op = CMD_OP_AC_ON;
//	else if (0 == strcmp( p, CMD_STR_AC_OFF ))
//		op = CMD_OP_AC_OFF;
//	else if (0 == strcmp( p, CMD_STR_WORK ))
//		op = CMD_OP_WORK;
//	else if (0 == strcmp( p, CMD_STR_SONARCONTROL ))
//		op = CMD_OP_SONARCONTROL;
//	else if (0 == strcmp( p, CMD_STR_ELEVATORUP ))
//		op = CMD_OP_ELEVATORUP;
//	else if (0 == strcmp( p, CMD_STR_ELEVATORDOWN ))
//		op = CMD_OP_ELEVATORDOWN;
//	else if (0 == strcmp( p, CMD_STR_ELEVATORTEST ))
//		op = CMD_OP_ELEVATORTEST;
//	else if (0 == strcmp( p, CMD_STR_ELEVATORSTOP ))
//		op = CMD_OP_ELEVATORSTOP;
//	else 
//		return -1;
//	for (int i = 0; i < MAX_CMD_PARAM_NUM; i++) {
//		p = strtok( 0, " " );
//		if (p == 0)
//			break;
//		pParam[(*nParamNum)++] = atoi( p );
//	}

//	return op;
}
