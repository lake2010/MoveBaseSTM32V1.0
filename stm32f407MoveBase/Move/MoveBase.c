#include "MoveBase.h"
#include "commen.h"
//#include "ICmd.h"
//#include "MyCmd.h"
CMoveBase_parameter MoveBase;

int		CMoveBase_procEvent( CMoveBase_parameter* CMoveBase_para, int nCmdOp, int* pParam, int nParamNum );	
void	CMoveBase_subLoop( CMoveBase_parameter* CMoveBase_para, int nCmdOp, const int* pParam, int nParamNum );
int		CMoveBase_AccStop( CMoveBase_parameter* CMoveBase_para, int vx, unsigned int Distance );
void	CMoveBase_checkHeartBeat( CMoveBase_parameter* CMoveBase_para);
void	TIM5_Init(void);
void	TIM4_Init(void);

void CMoveBase_setup( CMoveBase_parameter* CMoveBase_para )
{
	MoveBasePinDefine();
	CMoveBase_para->IRstate = 0;
	CMoveBase_para->softstop_state = -1;
	//总电源继电器pin初始化
	pinOutputModeInit(&m_PowerRelay);
	pinDigitalWrite(&m_PowerRelay, true);
	UP_ON_OFF = true;
	//软停开关初始化
	pinInputModeInit(&m_SoftStop_key);
	//开关机按键初始化
	pinInputModeInit(&m_Startup_Port);
	//PWM定时器设置
	TIM5_Init();
	TIM4_Init();
	//CCmd_setup( &CMoveBase_para->m_cmd );//message
	CMySerial_init(&CMoveBase_para->m_ccm);
	CMySerial_init(&CMoveBase_para->m_ccm3);
	CMove2_setup(&CMoveBase_para->m_move );//motor
	//CAutoCharge_setup( &CMoveBase_para->m_ac );//auto charge
	
	CODO2PG_setup( &CMoveBase_para->m_odo );//odo
	//CSonar_setup( &CMoveBase_para->m_sonar );//sonar
	CMoveBase_para->m_heartBeat = millis();
	CMoveBase_para->m_accTimer = millis();
	//CMove2_start( &CMoveBase_para->m_move );
	//CMove2_setv( &CMoveBase_para->m_move, 0, 0, 0 );
	COBD_setup(&CMoveBase_para->m_obd);	
	CLED_setup( &CMoveBase_para->m_LED, &m_LED_Port );
	SYSstatus_setup(&CMoveBase_para->m_SYSstatus);
	myprintf( "Boocax BooBase\r\n" );
	myprintf( "Keep moving, never losing!\r\n" );
}

//软件停止开关
void CMoveBase_softStop(CMoveBase_parameter* CMoveBase_para)
{
	if( pinDigitalRead(&m_SoftStop_key) == LOW ){
		delay(10);
		if( pinDigitalRead(&m_SoftStop_key) == LOW ){
			myprintf("LOW\r\n");
			CMove2_setv(&CMoveBase_para->m_move, 0, 0, 0);
			if( CMoveBase_para->softstop_state != 1 ){
				CMoveBase_para->softstop_state = 1;
				setbit(CMoveBase_para->SYSstatus,0);
				
			}
		}
	}else{
		if( CMoveBase_para->softstop_state != 0 ){
			CMoveBase_para->softstop_state = 0;
			clrbit(CMoveBase_para->SYSstatus,0);
			myprintf("HIGH\r\n");
		}
	}
}
void CMoveBase_checkStartUp(CMoveBase_parameter* CMoveBase_para)
{
	if(pinDigitalRead(&m_Startup_Port) == LOW){
		delay(10);
		if(pinDigitalRead(&m_Startup_Port) == LOW){
			while(pinDigitalRead(&m_Startup_Port) == LOW);
			if(UP_ON_OFF){
				UP_ON_OFF = false;
				//g_obj.pMove->stop();
				CMove2_setv(&CMoveBase_para->m_move, 0, 0, 0);
				delay(60000);
				pinDigitalWrite(&m_PowerRelay, LOW);
			}else{
				CMove2_start( &CMoveBase_para->m_move );
				UP_ON_OFF = true;
				pinDigitalWrite(&m_PowerRelay,  HIGH);
				delay(5000);
			}
		}
	}
}


void CMoveBase_USART4Read(CMoveBase_parameter* CMoveBase_para)
{
	//急停状态不响应指令
	if(CMoveBase_para->softstop_state == 1)
		return;
	int16_t pParam[MAX_CMD_PARAM_NUM];
	char*    buf = 0;
	int      len;
	if( CMySerial_getCmd_reeman(&CMoveBase_para->m_ccm, &buf, &len) ){
		unsigned char crc = 0;
		for( int i = 2; i < (len-1); i++ ){
			crc ^=buf[i];
		}
		if( crc != buf[len-1] )
			return;	
		CMoveBase_para->m_LED.m_canLedBlink = true;
		CMoveBase_para->getcmd_state = 1;//获得有效指令
		switch(buf[3])
		{
			case CMD_OP_START:
				CMove2_start(&CMoveBase_para->m_move);
				myprintfUSART1("CMD_OP_START\r\n");
				break;
			case CMD_OP_SET_V:	
				pParam[0] = ((int16_t)buf[4]<<8)   |
							((int16_t)buf[5]);
				pParam[1] = ((int16_t)buf[6]<<8)   |
							((int16_t)buf[7]);
				pParam[2] = ((int16_t)buf[8]<<8)   |
							((int16_t)buf[9]);
				//myprintfUSART1("%d,%d,%d",pParam[0],pParam[1],pParam[2]);
				//myprintfUSART4("%d,%d,%d",pParam[0],pParam[1],pParam[2]);
				myprintfUSART1("CMD_OP_SET_V\r\n");
				CMove2_setv( &CMoveBase_para->m_move, pParam[0],pParam[1],pParam[2]);				
				break;
			case CMD_OP_STOP: /* 电机停止运动（电机停止后，保持使能,但可被转动） */	
				CMove2_stop( &CMoveBase_para->m_move);
				myprintfUSART1("CMD_OP_STOP\r\n");
				break;
			case CMD_OP_AC_ON:
				CMove2_enableCoff( &CMoveBase_para->m_move, 1 );
				//接收到充电指令，重新发送运动控制板
				CMoveBase_para->getCoff_state = 1;
				mySerialWriteUSART3((byte*)buf,len);
				mySerialWriteUSART1((byte*)buf,len);
				break;
			case CMD_OP_AC_OFF:
				CMove2_enableCoff( &CMoveBase_para->m_move, 0 );
				mySerialWriteUSART3((byte*)buf,len);
				CMoveBase_para->getCoff_state = 0;
				myprintfUSART1("CMD_OP_AC_OFF\r\n");
				break;
			default:
				//mySerialWriteUSART4((byte*)buf,len);
				break;
		}
		//mySerialWriteUSART4((byte*)buf,len);
	}
	else
	{
		CMoveBase_para->getcmd_state  = 0;
	}
}

void CMoveBase_USART3Read(CMoveBase_parameter* CMoveBase_para)
{
	//急停状态不响应指令
	if(CMoveBase_para->softstop_state == 1)
		return;
	int16_t pParam[MAX_CMD_PARAM_NUM];
	char*    buf = 0;
	int      len;
	if( CMySerial_getCmd_reeman(&CMoveBase_para->m_ccm3, &buf, &len) ){
		unsigned char crc = 0;
		for( int i = 2; i < (len-1); i++ ){
			crc ^=buf[i];
		}
		if( crc != buf[len-1] )
			return;	
		CMoveBase_para->m_LED.m_canLedBlink = true;
		CMoveBase_para->getcmd_state = 1;//获得有效指令
		switch(buf[3])
		{
			case CMD_OP_START:
				CMove2_start(&CMoveBase_para->m_move);
				//myprintfUSART4("\nCMD_OP_START\n");
				break;
			case CMD_OP_SET_V:	
				pParam[0] = ((int16_t)buf[4]<<8)   |
							((int16_t)buf[5]);
				pParam[1] = ((int16_t)buf[6]<<8)   |
							((int16_t)buf[7]);
				pParam[2] = ((int16_t)buf[8]<<8)   |
							((int16_t)buf[9]);
				//myprintfUSART1("%d,%d,%d",pParam[0],pParam[1],pParam[2]);
				//myprintfUSART4("%d,%d,%d",pParam[0],pParam[1],pParam[2]);
				CMove2_setv( &CMoveBase_para->m_move, pParam[0],pParam[1],pParam[2]);				
				break;
			case CMD_OP_STOP: /* 电机停止运动（电机停止后，保持使能,但可被转动） */	
				CMove2_stop( &CMoveBase_para->m_move);
				break;
			case CMD_OP_AC_ON:
				CMove2_enableCoff( &CMoveBase_para->m_move, 1 );
				break;
			case CMD_OP_AC_OFF:
				CMove2_enableCoff( &CMoveBase_para->m_move, 0 );
				break;
			default:
				break;
		}
	}
	else
	{
		CMoveBase_para->getcmd_state  = 0;
	}
}



void CMoveBase_USART4ReadOff(CMoveBase_parameter* CMoveBase_para)
{
	//急停状态不响应指令
	if(CMoveBase_para->softstop_state == 1)
		return;
	char*    buf = 0;
	int      len;
	if( CMySerial_getCmd_reeman(&CMoveBase_para->m_ccm, &buf, &len) ){
		unsigned char crc = 0;
		for( int i = 2; i < (len-1); i++ ){
			crc ^=buf[i];
		}
		if( crc != buf[len-1] )
			return;	
		CMoveBase_para->m_LED.m_canLedBlink = true;
		CMoveBase_para->getcmd_state = 1;//获得有效指令
		switch(buf[3])
		{
			case CMD_OP_AC_OFF:
				CMove2_enableCoff( &CMoveBase_para->m_move, 0 );
				CMoveBase_para->getCoff_state = 0;
				mySerialWriteUSART3((byte*)buf,len);
			    myprintfUSART1("OFF\r\n");
				break;
			default:
				break;
		}

	}
	else
	{
		CMoveBase_para->getcmd_state  = 0;
	}
}

void CMoveChargeState_loop(CMoveBase_parameter* CMoveBase_para)
{
	if(CMoveBase_para->getCoff_state)
	{
		CMoveBase_USART4ReadOff(CMoveBase_para);	
		CMoveBase_USART3Read(CMoveBase_para);
	}
	else
		CMoveBase_USART4Read(CMoveBase_para);
	
}

void SYSstatus_setup( SYSstatus_parameter* SYSstatus_para )
{
	SYSstatus_para->charge_status = 0;
	SYSstatus_para->driver_status = 0;	
	SYSstatus_para->IO_status = 0;		
	SYSstatus_para->Sys_status = 0;		
	SYSstatus_para->m_SYStimeLastRead = millis();
}



void SYSstatus_printfEncoding(char value1, char value2, char value3, char value4)
{
	byte buf[20];
	int index = 0;
	buf[index++] = 0xAA;
	buf[index++] = 0x55;
	buf[index++] = 0x01;//len
	buf[index++] = 0x6F;
	buf[index++] = value1;
	buf[index++] = value2;
 	buf[index++] = value3;
	buf[index++] = value4;
	buf[2] = index-3;//len
	byte crc = 0;
	for(int i = 2; i < index; i++)
		crc ^= buf[i];
	buf[index++] = crc;
	mySerialWriteUSART4(buf,index); //主机
	//mySerialWriteUSART3(buf,index); //
}



void SYSstatus_loop( SYSstatus_parameter* SYSstatus_para )
{
	char*    buf = 0;
	int      len;
	unsigned long dt = millis() - SYSstatus_para->m_SYStimeLastRead; 
	if (dt>800)
	{ 
		if(CMySerial_getCmd_reeman(&MoveBase.m_ccm3, &buf, &len) )
		{
			unsigned char crc = 0;
			for( int i = 2; i < (len-1); i++ ){
				crc ^=buf[i];
			}
			if( crc != buf[len-1] )
			return;	
			switch(buf[3])
			{
				case CMD_OP_SYS_STATUS:
					MoveBase.m_ac.m_chargeState = buf[4];
					break;
			}
			
		}
		SYSstatus_printfEncoding(MoveBase.m_ac.m_chargeState, SYSstatus_para->driver_status, SYSstatus_para->IO_status, SYSstatus_para->Sys_status);
		//MODslave_Send_0DH( MoveBase.m_ac.m_chargeState, SYSstatus_para->driver_status, SYSstatus_para->IO_status, SYSstatus_para->Sys_status, CMD_OP_SYS_STATUS);
	    SYSstatus_para->m_SYStimeLastRead = millis();
	}
}


//运动控制板只相应动作指令
void CMoveBase_loop( CMoveBase_parameter* CMoveBase_para )
{
//	CMoveBase_softStop(CMoveBase_para);   //改好
	CMoveChargeState_loop(CMoveBase_para);//差更改充电指令
//	CODO2PG_loop( &CMoveBase_para->m_odo);//验证发送ODO指令
//	COBD_loop(&CMoveBase_para->m_obd);		//验证电压
	CLED_loop( &CMoveBase_para->m_LED);		//验证灯逻辑
	SYSstatus_loop( &CMoveBase_para->m_SYSstatus);//系统状态发送
//	CMoveBase_checkHeartBeat( CMoveBase_para);
//	CMoveBase_checkStartUp(CMoveBase_para);
}

int CMoveBase_procEvent( CMoveBase_parameter* CMoveBase_para, int nCmdOp, int* pParam, int nParamNum )
{/*注释 还没写到
	int sonarEvent = m_sonar.getEvent();
	unsigned int minDistance;
	int sonarWaring = m_sonar.getWarning( minDistance );
	int vx = 0, vy = 0, wz = 0;
	g_obj.pODO->getv( vx, vy, wz );
	if( vx > 600 )
		vx = 600;
	if (sonarEvent > 0) {
		sonarprintf(( "in movebace event\n" ));
		if( sonarWaring ){
			if( vx > 0 || pParam[0] > 0 ){
				g_obj.pMove->stop();
				delay( 300 );
				g_obj.pMove->start();
				pParam[0] = 0;
				pParam[1] = 0;
				pParam[2] = 0;
				memset( pParam, 0, nParamNum * sizeof(int) );
				return CMD_OP_SET_V;
			}					
		}else {
			switch (nCmdOp) {
				case CMD_OP_SET_V:
				case CMD_OP_SET_V2:
					if (nParamNum == 3) {
						if (pParam[0] > 0) {
							pParam[0] = vx / 20;
							if (  0 < pParam[0] && pParam[0]<= 50 ){
								pParam[0] = 0;
								g_obj.pMove->stop();
								delay( 300 );
								g_obj.pMove->start();
							}
							return CMD_OP_SET_V;
						}
					}
					break;
				default:
					break;
			}
		}
	}
	return nCmdOp;
	*/
	return 0;
}

int CMoveBase_AccStop( CMoveBase_parameter* CMoveBase_para, int vx, unsigned int Distance )
{
	int Acc = vx * vx * 10 / 2 / Distance;
	return Acc;
}

void CMoveBase_subLoop( CMoveBase_parameter* CMoveBase_para, int nCmdOp, const int* pParam, int nParamNum )
{
	//CAutoCharge_loop( &CMoveBase_para->m_ac, &nCmdOp, pParam, nParamNum );
	//CSonar_loop( &CMoveBase_para->m_sonar, nCmdOp, pParam, nParamNum );
	//CMove2_loop( &CMoveBase_para->m_move, nCmdOp, pParam, nParamNum );
	//CODO2PG_loop( &CMoveBase_para->m_odo, nCmdOp, pParam, nParamNum ); //编码器
	//COBD_loop(&CMoveBase_para->m_obd, nCmdOp, pParam, nParamNum);
	//CLED_loop( &CMoveBase_para->m_LED, nCmdOp, pParam, nParamNum);
}

void CMoveBase_checkHeartBeat( CMoveBase_parameter* CMoveBase_para)
{
	if( CAutoCharge_isChargeing( &CMoveBase_para->m_ac ) ){
		CMoveBase_para->m_heartBeat = millis();
		return;
	}
	if (CMoveBase_para->getcmd_state)
		CMoveBase_para->m_heartBeat = millis();
	else if (millis() - CMoveBase_para->m_heartBeat > 1000) {
		CMove2_setv( &CMoveBase_para->m_move, 0, 0, 0 );
		//int pParam[] = {0, 0, 0};
		CODO2PG_loop( &CMoveBase_para->m_odo);
		CMoveBase_para->m_heartBeat = millis();
	}
}
//PC6 LEFT PC7 RIGHT
void TIM5_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //GPIOA8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//
	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
	
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA8

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM5);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_TimeBaseStructure.TIM_Prescaler=8-1; 
	//TIM_TimeBaseStructure.TIM_Prescaler=21-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=2000-1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//初始化定时器9
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_Pulse=0;

	TIM_OC3Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM9 OC2
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM9在CCR2上的预装载寄存器
	
	TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPE使能,这样不会打乱计数器计数
	TIM_SetCompare3(TIM5,1000-1);
	//TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	//TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Enable);
	TIM_Cmd(TIM5, DISABLE);
}

void TIM4_Init(void)
{
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //GPIOA8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化PA8

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_TimeBaseStructure.TIM_Prescaler=8-1; 
	//TIM_TimeBaseStructure.TIM_Prescaler=21-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=2000-1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器9

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_Pulse=0;

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM9 OC2
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM9在CCR2上的预装载寄存器

	TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能,这样不会打乱计数器计数
	TIM_SetCompare1(TIM4,1000-1);
	//TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	//TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Enable);
	TIM_Cmd(TIM4, DISABLE);
}
