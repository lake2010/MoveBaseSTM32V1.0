#include "AutomaticCharge.h"
#include "cmd.h"
#include "MoveBase.h"
extern boolean UP_ON_OFF; //开关机标志

int CAutoCharge_getInfrared(CAutoCharge_parameter* CAutoCharge_para);

void TIM9_interruptInit(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);

	TIM_TimeBaseInitStructure.TIM_Period = 200-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler=42-1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 

	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseInitStructure);

	TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel=TIM1_BRK_TIM9_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM9,ENABLE);
}

float CAutoCharge_getContactVoltage(void)
{
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_480Cycles );//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    

	ADC_SoftwareStartConv(ADC1);//使能指定的ADC1的软件转换启动功能	

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	float val = ADC_GetConversionValue(ADC1);//返回最近一次ADC1规则组的转换结果

	return val / (float)4095.00 * (float)3.30;
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM9,TIM_IT_Update)==SET) //溢出中断
	{
		CAutoCharge_interrupt(&MoveBase.m_ac);
	}
	TIM_ClearITPendingBit(TIM9,TIM_IT_Update);
	
}

void CAutoCharge_interrupt(CAutoCharge_parameter* CAutoCharge_para)
{
	IRSendRev_myinterrupt(&CAutoCharge_para->myIR[0]);
	IRSendRev_myinterrupt(&CAutoCharge_para->myIR[1]);	
}

void CAutoCharge_setup(CAutoCharge_parameter* CAutoCharge_para)
{
	CAutoCharge_para->m_chargeState = charge_IDEL;
	CAutoCharge_para->m_isChargeing = 0;

	pinOutputModeInit(&m_Relay_AC);
	pinDigitalWrite(&m_Relay_AC, !m_AC_CHARGE);
	TIM9_interruptInit();//红外50us定时中断

	IRSendRev_Init(&CAutoCharge_para->myIR[0], &m_IR_Port_L);//内部包含引脚定义
	IRSendRev_Init(&CAutoCharge_para->myIR[1], &m_IR_Port_R);//内部包含引脚定义

	CAutoCharge_stop(CAutoCharge_para);
}

void CAutoCharge_start(CAutoCharge_parameter* CAutoCharge_para)
{
	/*ding shi qi kai */
	CMove2_start(&MoveBase.m_move);
	TIM_Cmd(IR_TIMER,ENABLE);
	CAutoCharge_para->m_isChargeing = 1;
	CAutoCharge_para->m_chargeState = charge_testIRsignal;
	CAutoCharge_para->m_testIRsignalcnt = 0;
	CAutoCharge_para->m_sytemTime = millis();
}

void CAutoCharge_stop(CAutoCharge_parameter* CAutoCharge_para)
{
	CAutoCharge_para->m_chargeState = charge_IDEL;
	CAutoCharge_para->m_isChargeing = 0;
	//定时器关
	TIM_Cmd(IR_TIMER,DISABLE);
}

void CAutoCharge_loop( CAutoCharge_parameter* CAutoCharge_para, int *nCmdOp, const int *pParam, int nParamNum )
{
	//暂先注释CODO2PG_loop( &MoveBase.m_odo, *nCmdOp, pParam, nParamNum );
	//-------------------------------------------------------
	if( *nCmdOp == CMD_OP_AC_ON ){
		if( CAutoCharge_para->m_isChargeing == 0 ){
			if( COBD_getStableVol(&MoveBase.m_obd) > (float)28.5 ){//g_obj.pTemp->get_outVol() > 28.5 ){
				myprintf( "@chargeerror5#\r\n" );//不需要充电
			}else{
				CAutoCharge_start(CAutoCharge_para);
				CAutoCharge_para->m_isChargeing =  1;
			}		
		}
	}else if ( *nCmdOp == CMD_OP_AC_OFF ){
		CMove2_setv( &MoveBase.m_move, 0, 0, 0 );
		CAutoCharge_stop(CAutoCharge_para);
		return;
	}

	if( CAutoCharge_para->m_isChargeing == 1 )
		*nCmdOp = -1;
	//-------------------------------------------------------
	switch (CAutoCharge_para->m_chargeState)
	{
		case charge_IDEL:
			break;
		case charge_testIRsignal:
			if( millis() - CAutoCharge_para->m_sytemTime <= m_testIRsignal ){//2s内检测红外信号个数
				if( CAutoCharge_getInfrared(CAutoCharge_para) != -1 )
					CAutoCharge_para->m_testIRsignalcnt++;
				delay(5);
			}else if( CAutoCharge_para->m_testIRsignalcnt < 6 ){//信号不正常
				CAutoCharge_stop(CAutoCharge_para);
				myprintf( "@chargeerror4#\r\n" );//发送充电命令，2s内检测红外信号，信号不稳定
			}else{//信号正常
				CAutoCharge_para->m_chargeState = charge_gotoCharge;
				CAutoCharge_para->m_sytemTime = millis();
				CAutoCharge_para->m_chargeUsingTime = millis();
			}
			break;
		case charge_gotoCharge:
			switch (CAutoCharge_getInfrared(CAutoCharge_para))
			{
				case -1://no one
					printddd(("no one\r\n"));
					break;
				case 0://only right
					CAutoCharge_para->m_sytemTime = millis();
					CMove2_setv( &MoveBase.m_move, -80, 0, 200 );
					printddd(("noly right\r\n"));
					break;
				case 1://only left
					CAutoCharge_para->m_sytemTime = millis();
					CMove2_setv( &MoveBase.m_move, -80, 0, -200 );
					printddd(("noly left\r\n"));
					break;
				case 2://all
					CAutoCharge_para->m_sytemTime = millis();
					CMove2_setv( &MoveBase.m_move, -80, 0, 0 );
					printddd(("go go go\r\n"));
					break;
				default:
					break;
			}
			if( millis() - CAutoCharge_para->m_sytemTime > m_receiveIRdetTime ){//检测运动中是否没有接收到信号
				CAutoCharge_stop(CAutoCharge_para);
				myprintf( "@chargeerror1#\r\n" );//m_IR_STOP_COUNT时间内没有收到IR信号，信号被挡住或者发射、接收头损坏
			}
			
			if( CAutoCharge_getContactVoltage() > (float)1.65 ) {//检测到信号
				//延时向前进
				delay(50);
				pinDigitalWrite( &m_Relay_AC, m_AC_CHARGE );//切换为充电模式
				CMove2_stop(&MoveBase.m_move);
				CAutoCharge_para->m_chargeState = charge_testCharge;
				CAutoCharge_para->m_sytemTime = millis();
			}else if( millis() - CAutoCharge_para->m_chargeUsingTime > m_WaitShutDownTime ){
				CAutoCharge_stop(CAutoCharge_para);
				myprintf( "@chargeerror2#\r\n" );//规定最长对接时间内没有完成对接
			}
			break;
		case charge_testCharge:
			if( millis() - CAutoCharge_para->m_sytemTime >= 10000 ){
				pinDigitalWrite( &m_Relay_AC, !m_AC_CHARGE );//检测有无接触
				if ( CAutoCharge_getContactVoltage() > (float)1.65 ){
					pinDigitalWrite( &m_Relay_AC, m_AC_CHARGE );//继续充电
					myprintf("@chargeok#\r\n");//完全对接成功，开始延时60s，等待上位机关机
					TIM_Cmd(IR_TIMER,DISABLE);//关闭定时器...但还在充电中
					CAutoCharge_para->m_chargeState = charge_waitShutdown;//进入等待上位机关机延时
					CAutoCharge_para->m_sytemTime = millis();
				}else{
					CAutoCharge_stop(CAutoCharge_para);
					myprintf( "@chargeerror3#\r\n" );//初步对接成功后，十秒钟后再次检测，对接异常,并向前行走10cm
				}
			}
			break;
		case charge_waitShutdown:
			if( millis() - CAutoCharge_para->m_sytemTime > m_WaitShutDownTime ){
				//digitalWrite(LED_KEY, LOW);//关闭led
				pinDigitalWrite(&m_PowerRelay, false);//关闭固态继电器
				UP_ON_OFF = false;//关机标志位变为false
				CAutoCharge_para->m_chargeState = charge_gotoWork;
				CAutoCharge_para->m_sytemTime = millis();
			}
			break;
		case  charge_gotoWork:
			if( COBD_getStableVol(&MoveBase.m_obd) > (float)29 ){//Vol > 29 ){
				CAutoCharge_para->m_sytemTime = millis();
				pinDigitalWrite( &m_Relay_AC, !m_AC_CHARGE );
				pinDigitalWrite(&m_PowerRelay, true);//充电完成打开总继电器
				CMove2_start(&MoveBase.m_move);
				CMove2_setv( &MoveBase.m_move, 100, 0, 0 );
				while( millis() - CAutoCharge_para->m_sytemTime < 1000 )
					CSonar_loop(&MoveBase.m_sonar,0,0,0);
				CMove2_setv( &MoveBase.m_move, 0, 0, 0 );
				CAutoCharge_stop(CAutoCharge_para);
				UP_ON_OFF = true;
			}else if( millis() - CAutoCharge_para->m_sytemTime > m_chargeDetTime ){
				CAutoCharge_para->m_sytemTime = millis();
				pinDigitalWrite( &m_Relay_AC, !m_AC_CHARGE );
				if ( CAutoCharge_getContactVoltage() > (float)1.65 ){
					pinDigitalWrite( &m_Relay_AC, m_AC_CHARGE );//对接正常，继续充电
					CAutoCharge_para->m_sytemTime = millis();
				}else{
					CAutoCharge_stop(CAutoCharge_para);
					myprintf( "@chargeerror5#\r\n" );//充电过程中脱落
				}
			}
			break;
		default:
			break;
	}
}

int  CAutoCharge_isGotoWork(CAutoCharge_parameter* CAutoCharge_para)
{
	if( CAutoCharge_para->m_chargeState == charge_gotoWork )
		return 1;
	else return 0;
}

int  CAutoCharge_isChargeing(CAutoCharge_parameter* CAutoCharge_para)
{
	return CAutoCharge_para->m_isChargeing;
}

int CAutoCharge_getInfrared(CAutoCharge_parameter* CAutoCharge_para)
{
	int a = -1;
	unsigned char dta[20]; 
	if(IRSendRev_IsDta(&CAutoCharge_para->myIR[0])){
        int length= IRSendRev_Recv(&CAutoCharge_para->myIR[0], dta);
		for(int j = 0; j < length; j++)
			printddd(("L  %d",dta[j]));
		printddd(("\r\n"));
		delay(5);
		if ( 4==dta[5] && 
			( m_INFRARED_RECIEVED>>24      )==dta[6] && 
			((m_INFRARED_RECIEVED>>16)&0xFF)==dta[7] && 
			((m_INFRARED_RECIEVED>>8 )&0xFF)==dta[8] && 
			((m_INFRARED_RECIEVED    )&0xFF)==dta[9] ){
          a = 1;//数据正确-标志位置一
        }
    }
	if(IRSendRev_IsDta(&CAutoCharge_para->myIR[1])){
        int length= IRSendRev_Recv(&CAutoCharge_para->myIR[1], dta);
		for(int j = 0; j < length; j++)
			printddd(("R  %d",dta[j]));
		printddd(("\r\n"));
		delay(5);
        if ( 4==dta[5] && 
			( m_INFRARED_RECIEVED>>24)      ==dta[6] && 
			((m_INFRARED_RECIEVED>>16)&0xFF)==dta[7] && 
			((m_INFRARED_RECIEVED>>8 )&0xFF)==dta[8] && 
			((m_INFRARED_RECIEVED    )&0xFF)==dta[9] ){
          a += 1;
        }
    }
  return a;
}
