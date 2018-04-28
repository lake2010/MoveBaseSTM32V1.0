#include "cmyserial.h"
#include "MyCmd.h"

CMySerial_parameter CCmd;

void CMySerial_analysisCMD(CMySerial_parameter* CMySerial_para, char* buf, int* len);

void CMySerial_init(CMySerial_parameter* CMySerial_para)
{
	CMySerial_para->m_receive_index = 0;
	CMySerial_para->m_receive_head = 0;
	CMySerial_para->m_receive_tail = 0;
}

void CMySerial_addCmdtoOrignal_buff(CMySerial_parameter* CMySerial_para, char readBuff)
{
	CMySerial_para->m_receive_orignal_buff[CMySerial_para->m_receive_head++] = readBuff;
	if( CMySerial_para->m_receive_head >= MAX_RECEIVE_ORIGNAL_BUFLEN )
		CMySerial_para->m_receive_head = 0;
}

boolean CMySerial_getCmd_reeman(CMySerial_parameter* CMySerial_para, char** buf, int* len)
{
	while( CMySerial_para->m_receive_tail != CMySerial_para->m_receive_head ){
		CMySerial_para->m_receive_buff[CMySerial_para->m_receive_index] = CMySerial_para->m_receive_orignal_buff[CMySerial_para->m_receive_tail];
		if( ++CMySerial_para->m_receive_tail >= MAX_RECEIVE_ORIGNAL_BUFLEN )
			CMySerial_para->m_receive_tail = 0;
		if( CMySerial_para->m_receive_index == 0 ){
			if( (byte)CMySerial_para->m_receive_buff[CMySerial_para->m_receive_index] == 0xAA ){
				CMySerial_para->m_receive_index++;
			}
		}else if( CMySerial_para->m_receive_index == 1 ){
			if( (byte)CMySerial_para->m_receive_buff[CMySerial_para->m_receive_index] == 0x55 ){
				CMySerial_para->m_receive_index++;
			}else if( (byte)CMySerial_para->m_receive_buff[CMySerial_para->m_receive_index] != 0xAA ){
				CMySerial_para->m_receive_index = 0;
			}
		}else{
			if( CMySerial_para->m_receive_index == (unsigned char)CMySerial_para->m_receive_buff[2]+3 ){
				*buf = CMySerial_para->m_receive_buff;
				*len = CMySerial_para->m_receive_index + 1;
				CMySerial_para->m_receive_index = 0;
				return true;
			}else if( CMySerial_para->m_receive_index >= MAX_RECEIVE_BUFLEN ){
				CMySerial_para->m_receive_index = 0;
			}else{
				CMySerial_para->m_receive_index++;
			}
		}
	}
	return false;
}

void CMySerial_analysisCMD(CMySerial_parameter* CMySerial_para, char* buf, int* len)
{
	unsigned char crc = 0;
	for( int i = 2; i < (*len-1); i++ ){
		crc ^=buf[i];
	}
	if( crc != buf[*len-1] )
		return;
	switch(buf[3]){
		case CMD_OP_START:	 //启动命令1
				
		break;

		default:
			break;
	}
}

