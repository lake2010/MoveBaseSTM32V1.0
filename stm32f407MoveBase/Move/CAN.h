#ifndef _CAN_H
#define _CAN_H
#include "commen.h"
#include "cmyserial.h"
typedef struct CCAN_parameter_all
{
	CMySerial_parameter Can_ccm; 
	CanTxMsg TxMessage;
	CanRxMsg RxMessage;
	
}CCAN_parameter;

u8 CSonor_can1ModeInit(void);
u8 CSonor_can1SendMsg(u8* msg,u8 len);
void CSonor_can1SendMsgNum(u8* msg,u8 len);
u8 CSonor_can1ReceiveMsg(void);

#endif
