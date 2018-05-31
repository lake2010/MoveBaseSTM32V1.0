#include "motecSerial.h"
#include "config.h"
#include "stmflash.h"

//void	CMySerial_analysisCMD(CMySerial_parameter* CMySerial_para, char* buf, int* len);
/*********************************串口协议接收**********************************************/
uint16_t MotecSerial_generateXOR(byte* buf)
{
	uint16_t mxor = 0;
	uint16_t data[(MOTECSERIAL_LEN)/2];
	for( int i = 0; i < (MOTECSERIAL_LEN); ){
		data[i/2] = (uint16_t)buf[i] << 8 | (uint16_t)buf[i+1];
		if( i/2 <= (MOTECSERIAL_LEN)/2-2 )
			mxor ^= data[i/2];
		i = i+2;
	}
	return mxor;
}

void MotecSerial_sendCommand(uint8_t comID, uint32_t sData)
{
	uint16_t mxor = 0;
	byte sendBuf[MOTECSERIAL_LEN];
	int index = 0;
	sendBuf[index++] = MOTOR_ADDRESS;
	sendBuf[index++] = comID;
	for( int i = (MOTECSERIAL_LEN/2-1); i >= 0; i-- ){
		sendBuf[index++] = (byte)((sData >> (i*8)) & 0xFF);
	}
	mxor = MotecSerial_generateXOR(sendBuf);
	sendBuf[index++] = (byte)(mxor >> 8) & 0xFF;
	sendBuf[index++] = (byte)(mxor) & 0xFF;
	mySerialWriteUSART6(sendBuf, index);
	//mySerialWriteUSART2(sendBuf, index);
}

void MotecSerial_setStep(uint8_t comID, uint32_t sData)
{
	MotecSerial_sendCommand(comID, sData);
	MotecSerial_sendCommand(MOTEC_Go, 0x00000000);
}

int MotecSerial_checkXOR(MotecSerial_parameter* CMySerial_para, char* buf)
{
	uint16_t xor = 0;
	uint16_t data[(MOTECSERIAL_LEN)/2];
	for( int i = 0; i < (MOTECSERIAL_LEN); ){
		data[i/2] = (uint16_t)buf[i] << 8 | (uint16_t)buf[i+1];
		if( i/2 <= (MOTECSERIAL_LEN)/2-2 )
			xor ^= data[i/2];
		i = i+2;
	}
	if( xor == data[(MOTECSERIAL_LEN)/2-1] )
		return true;
	else
		return false;
}

void MotecSerial_deletOneByte(MotecSerial_parameter* CMySerial_para, char* buf, int index)
{
	for( int i = 1; i < index; i++ ){
		buf[i-1] = buf[i];
	}
}

void MotecSerial_init(MotecSerial_parameter* CMySerial_para)
{
	CMySerial_para->m_receive_index = 0;
	CMySerial_para->m_receive_head = 0;
	CMySerial_para->m_receive_tail = 0;
}

void MotecSerial_addCmdtoOrignal_buff(MotecSerial_parameter* CMySerial_para, char readBuff)
{
	CMySerial_para->m_receive_orignal_buff[CMySerial_para->m_receive_head++] = readBuff;
	if( CMySerial_para->m_receive_head >= MAX_RECEIVE_ORIGNAL_BUFLEN )
		CMySerial_para->m_receive_head = 0;
}

boolean MotecSerial_getCmd(MotecSerial_parameter* CMySerial_para, char** buf, int* len)
{
	while( CMySerial_para->m_receive_tail != CMySerial_para->m_receive_head ){
		/*有数据，并获取最新数据*/
		CMySerial_para->m_receive_buff[CMySerial_para->m_receive_index++] = 
			CMySerial_para->m_receive_orignal_buff[CMySerial_para->m_receive_tail];
		/*尾部自增，以获取新数据*/
		if( ++CMySerial_para->m_receive_tail >= MAX_RECEIVE_ORIGNAL_BUFLEN )
			CMySerial_para->m_receive_tail = 0;
		/*接收数据满足 协议固定长度*/
		if( CMySerial_para->m_receive_index == MOTECSERIAL_LEN ){
			/*判断头*/
			if( CMySerial_para->m_receive_buff[0] == MOTOR_ADDRESS ){
				/*头符合条件验证异或校验位*/
				if(MotecSerial_checkXOR(CMySerial_para, CMySerial_para->m_receive_buff)){
					/*校验通过*/
					*buf = CMySerial_para->m_receive_buff;
					*len = CMySerial_para->m_receive_index;
					CMySerial_para->m_receive_index = 0;
					return true;
				}
			}
			MotecSerial_deletOneByte(CMySerial_para, CMySerial_para->m_receive_buff, CMySerial_para->m_receive_index);
			CMySerial_para->m_receive_index--;
			return false;
		}
	}
	return false;
}
/*************************************串口协议接收 END***********************************************/
