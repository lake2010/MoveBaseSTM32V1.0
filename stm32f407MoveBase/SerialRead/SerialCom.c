#include "SerialCom.h"
#include "config.h"
void CSerialCom_recv( CSerialCom_param* CSerialCom_para )
{
	int16_t x_from_serial;
	int cmd_len;
	char cmd[MAX_CMD_STRLEN];
	while (1) {
		x_from_serial = (int16_t)readFromUart();
		if (x_from_serial == -1)
			break;
		CSerialBuf_AddByte(&CSerialCom_para->m_SerialBuf, (char)x_from_serial );
		cmd_len = CSerialBuf_GetPackage(&CSerialCom_para->m_SerialBuf, cmd, MAX_CMD_STRLEN-1 );
		if (cmd_len > 0) {
			cmd[cmd_len] = 0;
			if (CSerialCom_para->m_nCmdNum == m_MAX_CMD_NUM) {
				memcpy( CSerialCom_para->m_pCmd, CSerialCom_para->m_pCmd+1, (CSerialCom_para->m_nCmdNum-1)*sizeof(CMD_STRING) );
				CSerialCom_para->m_nCmdNum--;
			}
			strcpy( CSerialCom_para->m_pCmd[CSerialCom_para->m_nCmdNum].str, cmd );
			CSerialCom_para->m_nCmdNum++;
		}
	}
}

CMD_STRING* CSerialCom_getCmd( CSerialCom_param* CSerialCom_para, int *nCmdNum )
{
	*nCmdNum = CSerialCom_para->m_nCmdNum;
	CSerialCom_para->m_nCmdNum = 0;
	if (nCmdNum > 0)
		return CSerialCom_para->m_pCmd;
	else
		return 0;	
}

