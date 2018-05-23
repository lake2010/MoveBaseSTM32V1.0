#include "CAN.h"
#include "config.h"
#include "MoveBase.h"

u8 CSonor_can1ModeInit()
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); 
	
	//速率是500kbps
	CAN_InitStructure.CAN_TTCM=DISABLE;	 
	CAN_InitStructure.CAN_ABOM=DISABLE;	 
  CAN_InitStructure.CAN_AWUM=DISABLE;
  CAN_InitStructure.CAN_NART=ENABLE;	
  CAN_InitStructure.CAN_RFLM=DISABLE;	
  CAN_InitStructure.CAN_TXFP=DISABLE;	
  CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;	 //回环测试
  CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	
  CAN_InitStructure.CAN_BS1=CAN_BS1_7tq; 
  CAN_InitStructure.CAN_BS2=CAN_BS2_6tq;
  CAN_InitStructure.CAN_Prescaler=6;  
  CAN_Init(CAN1, &CAN_InitStructure); 
	
	CAN_FilterInitStructure.CAN_FilterNumber=0;	
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; 
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; 
  CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;           
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	return 0;
}

void CSonor_can1SendMsgNum(u8* msg,u8 len)
{	
	u8 *a;
	int msgLast = len % 8;
	int msgNum = len / 8; //分包发送
	if(msgNum > 0)
	{
		for(int i = 0;i < msgNum; i++)
		{
			a = msg + i * 8;
			CSonor_can1SendMsg(a,8);
		}
	}
	if(msgLast > 0)
	{
			a = msg + msgNum * 8;
			CSonor_can1SendMsg(a,msgLast);
	}
}

u8 CSonor_can1SendMsg(u8* msg,u8 len)
{
	u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x12;	 
  TxMessage.ExtId=0x12;	
  TxMessage.IDE=0;		  
  TxMessage.RTR=0;		 
  TxMessage.DLC=len;					
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];		       
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	
  if(i>=0XFFF)return 1;
  return 0;		
}
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
  CAN_Receive(CAN1, 0, &RxMessage);
	for(int i = 0;i < 8;i++)
	{
		char a = RxMessage.Data[i];
		CMySerial_addCmdtoOrignal_buff(&MoveBase.m_canccm.Can_ccm,a);
		//myprintfUSART1("%x ",a);
	}
}


