#include "ODO2PG.h"
#include "MoveBase.h"

uint16_t m_PROPORTION = 50;
void CODO2PG_couter2rad( CODO2PG_parameter* CODO2PG_para, const long* pCounter, int dt, long* pw );
void CODO2PG_moveModelForeward( CODO2PG_parameter* CODO2PG_para, const long* pw, int* vx, int* vy, int* wz );
void CODO2PG_interruptSetup(void);

void CODO2PG_setup( CODO2PG_parameter* CODO2PG_para )
{
	CODO2PG_interruptSetup();
	CODO2PG_para->m_vx = CODO2PG_para->m_vy = CODO2PG_para->m_wz = 0;
	memset( CODO2PG_para->m_pw, 0, m_MOTOR_NUM * sizeof(long) );
	CODO2PG_para->m_pwFrMove = 0;
	CODO2PG_para->m_counter_l = CODO2PG_para->m_counter_r = 0;
	CODO2PG_para->m_timeLastRead = millis();
	CODO2PG_para->m_start_time = millis();
	CODO2PG_para->m_odo_count = 0;
}
//只发USART4
void CODO2PG_printfEncoding(CODO2PG_parameter* CODO2PG_para,uint16_t dt)
{
	byte buf[20];
	int index = 0;
	buf[index++] = 0xAA;
	buf[index++] = 0x55;
	buf[index++] = 0x01;//len
	buf[index++] = 0x6E;
	buf[index++] = (( uint16_t)CODO2PG_para->m_vx>> 8) & 0xFF;
	buf[index++] = (((uint16_t)CODO2PG_para->m_vx) ) & 0xFF;
	buf[index++] = (( uint16_t)CODO2PG_para->m_vy>> 8) & 0xFF;
	buf[index++] = (((uint16_t)CODO2PG_para->m_vy))  & 0xFF;
	buf[index++] = (( uint16_t)CODO2PG_para->m_wz>> 8) & 0xFF;
	buf[index++] = (((uint16_t)CODO2PG_para->m_wz) ) & 0xFF;
	buf[index++] = (( uint16_t)dt>> 8) & 0xFF;
	buf[index++] = (((uint16_t)dt) ) & 0xFF;
	buf[2] = index-3;//len
	byte crc = 0;
	for(int i = 2; i < index; i++)
		crc ^= buf[i];
	buf[index++] = crc;
	mySerialWriteUSART4(buf,index);
//	mySerialWriteUSART1(buf,index);
}


void CODO2PG_loop( CODO2PG_parameter* CODO2PG_para)
{
	// get direction info from move object
	int nwFrMove;
	CODO2PG_para->m_pwFrMove = CMove2_getw( &MoveBase.m_move, &nwFrMove );
	
	unsigned long dt = millis() - CODO2PG_para->m_timeLastRead; // @@ int
	if (dt >= m_Ts) {
		long counter[2] = {CODO2PG_para->m_counter_l, CODO2PG_para->m_counter_r};
		CODO2PG_para->m_counter_l = CODO2PG_para->m_counter_r = 0;
		CODO2PG_para->m_timeLastRead = millis();
		if (dt < m_Ts + 1000) {
			//unsigned long det = millis();
			CODO2PG_couter2rad( CODO2PG_para, counter, dt, CODO2PG_para->m_pw );
			CODO2PG_moveModelForeward( CODO2PG_para, CODO2PG_para->m_pw, &CODO2PG_para->m_vx, &CODO2PG_para->m_vy, &CODO2PG_para->m_wz );
			//输出ODO指令
			CODO2PG_printfEncoding(CODO2PG_para,dt);
			//myprintf( "@odo %d %d %d %d#\r\n", CODO2PG_para->m_vx, CODO2PG_para->m_vy, CODO2PG_para->m_wz, dt );
			//myprintf( "det%d \n",(millis() - det) );
			CODO2PG_para->m_odo_count++;
		}
	}
}

void CODO2PG_getv( CODO2PG_parameter* CODO2PG_para, int* vx, int* vy, int* wz )
{
	*vx = CODO2PG_para->m_vx, *vy = CODO2PG_para->m_vy, *wz = CODO2PG_para->m_wz;
}

const long* CODO2PG_getw( CODO2PG_parameter* CODO2PG_para, int* n )
{
	*n = m_MOTOR_NUM;
	return CODO2PG_para->m_pw;
}

void CODO2PG_pg_l_int( CODO2PG_parameter* CODO2PG_para )
{
	if (CODO2PG_para->m_pwFrMove) {
		if (CODO2PG_para->m_pwFrMove[0] >= 0)
			CODO2PG_para->m_counter_l++;
		else
			CODO2PG_para->m_counter_l--;
	}
}

void CODO2PG_pg_r_int( CODO2PG_parameter* CODO2PG_para )
{
	if (CODO2PG_para->m_pwFrMove) {
		if (CODO2PG_para->m_pwFrMove[1] >= 0)
			CODO2PG_para->m_counter_r++;
		else
			CODO2PG_para->m_counter_r--;
	}
}

void CODO2PG_couter2rad( CODO2PG_parameter* CODO2PG_para, const long* pCounter, int dt, long* pw )
{
	//for (int i = 0; i < m_MOTOR_NUM; i++) {
	//	pw[i] = pCounter[i] * PI2000 * 20L / m_MAG_PRO; // couter to rad/1000
	//	pw[i] = pw[i] * 1000 / dt; // rad/1000 to rad/1000/s
	//}
	//for (int i = 0; i < m_MOTOR_NUM; i++) {
	//	pw[i] = pCounter[i] * PI2000 / m_WHEEL_DEC_NUM; // couter to rad/1000
	//	pw[i] = pw[i] * 1000 / dt; // rad/1000 to rad/1000/s
	//}

	for (int i = 0; i < m_MOTOR_NUM; i++) {
		pw[i] = (long)((double)pCounter[i] * (double)PI2000 * (double)20 / (double)m_MAG_PRO); // couter to rad/1000
		pw[i] = (long)((double)pw[i] * (double)1000 / (double)dt); // rad/1000 to rad/1000/s
		pw[i] = (long)((double)pw[i] / (double)60 / 2.0);
	}
}

void CODO2PG_moveModelForeward( CODO2PG_parameter* CODO2PG_para, const long* pw, int* vx, int* vy, int* wz )
{
	int pv[2];
	for (int i = 0; i < 2; i++) {
		pv[i] = (long)(((double)pw[i] * (double)m_WHEEL_R) / (double)1000);
	}
	*vx = (pv[0] + pv[1]) >> 1;
	*vy = 0;
	*wz = (long)((((double)pv[1] - (double)pv[0]) * (double)1000) / (double)m_WHEEL_D);
}

void CODO2PG_interruptSetup(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	//GPIO_InitTypeDef   GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//enable GPIOA clk
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//enable GPIOC clk
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line1 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling ;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

//left
void EXTI0_IRQHandler(void)
{
	if( EXTI_GetFlagStatus(EXTI_Line0) == !RESET ){
		CODO2PG_pg_l_int(&MoveBase.m_odo);
		EXTI_ClearITPendingBit(EXTI_Line0);
		return;
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
//	if (MoveBase.m_odo.m_pwFrMove) {
//		if (MoveBase.m_odo.m_pwFrMove[0] >= 0)
//			MoveBase.m_odo.m_counter_l++;
//		else
//			MoveBase.m_odo.m_counter_l--;
//	}
}
//right
void EXTI1_IRQHandler(void)
{
	if( EXTI_GetFlagStatus(EXTI_Line1) == !RESET ){
		CODO2PG_pg_r_int(&MoveBase.m_odo);
		EXTI_ClearITPendingBit(EXTI_Line1);
		return;
	}
	EXTI_ClearITPendingBit(EXTI_Line1);
//	if (MoveBase.m_odo.m_pwFrMove) {
//		if (MoveBase.m_odo.m_pwFrMove[1] >= 0)
//			MoveBase.m_odo.m_counter_r++;
//		else
//			MoveBase.m_odo.m_counter_r--;
//	}
}
