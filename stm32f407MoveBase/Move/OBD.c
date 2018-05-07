#include "OBD.h"

volatile static int64_t m_volCount = 0;

void ADC1_ch3ch7ch14ch15_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	//先初始化ADC1通道5 IO口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);


	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);  
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,  ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,DISABLE);


	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
//	ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 1, ADC_SampleTime_480Cycles  );
//	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 1, ADC_SampleTime_480Cycles );
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Cmd(ADC2, ENABLE);
}

void COBD_getElectricity(CODB_parameter* CODB_para)
{
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_480Cycles );//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    

	ADC_SoftwareStartConv(ADC1);//使能指定的ADC1的软件转换启动功能	

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	CODB_para->m_electricity 	= 25 /1.65 *  (ADC_GetConversionValue(ADC1)/4095 * 3.3 -1.65 );//返回最近一次ADC1规则组的转换结果

	//CODB_para->m_electricity = val / (float)4095.00 * (float)3.30;

}

////电压检测外部中断设置
//void COBD_Voltage_init(void)
//{
//	NVIC_InitTypeDef   NVIC_InitStructure;
//	EXTI_InitTypeDef   EXTI_InitStructure;

//	GPIO_InitTypeDef  GPIO_InitStructure;

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//enable GPIOB clk

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2);

//	/* 中断12*/
//	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//}

void COBD_getTemp(CODB_parameter* CODB_para)
{
	if(CODB_para->m_getTimpReady)//每轮测完一次就不测了
		return;
	static u8 TempID = 0;
	CODB_para->m_tmp[TempID] = DS18B20_getTemp(&CODB_para->m_DS18B20,TempID);
	if( CODB_para->m_tmp[TempID] < -20 || CODB_para->m_tmp[TempID] > 100 )
		CODB_para->m_tmp[TempID] = 0;
	TempID++;
	if( TempID >= CODB_para->m_DS18B20.DS18B20_Num ){
		TempID = 0;
		CODB_para->m_getTimpReady = true;
	}
}

void COBD_setup(CODB_parameter* CODB_para)
{
	for( int i = 0; i < m_TemperNum; i++ )
	CODB_para->m_tmp[i] = 0;	//-255为检测温度异常
	CODB_para->m_electricity = 0;
	CODB_para->m_voltage = 0;		//-1为检测电压异常
	CODB_para->m_batteryPercentage = 0;
	CODB_para->m_stableVol = 0;

	CODB_para->m_DS18B20.m_sensorNum = m_TemperNum;

	CODB_para->m_getTimpReady = false;
	CODB_para->m_volReadCount = 0;
	ADC1_ch3ch7ch14ch15_init();
	//更改方式
	//COBD_Voltage_init();
	DS18B20_setup(&CODB_para->m_DS18B20, &m_Temp_busPin, m_TemperNum);
	DS18B20_searchRom(&CODB_para->m_DS18B20);

	CODB_para->m_systemTime = millis();
	CODB_para->m_volGetTime = millis();
}

float COBD_getVol(int _t)
{
		ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 1, ADC_SampleTime_480Cycles  );//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度
		ADC_SoftwareStartConv(ADC2);//使能指定的ADC1的软件转换启动功能	
		while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ))
		{}
	  float val = ADC_GetConversionValue(ADC2)/ 4095 * 3.3 * 11;//返回最近一次ADC1规则组的转换结果   
		float y = val ;//1.028是为了调整精度
		return y;	
}

int COBD_getBatteryPercentage(float _vol)
{
 return  255;
}


void COBD_printfEncoding(uint16_t temp_voltage
						,uint16_t temp_batteryPercentage,uint16_t temp_electricity
						,uint16_t temp1_m_tmp,uint16_t temp2_m_tmp,uint16_t temp3_m_tmp)
{
	byte buf[20];
	int index = 0;
	buf[index++] = 0xAA;
	buf[index++] = 0x55;
	buf[index++] = 0x01;//len
	buf[index++] = 0x6F;
	buf[index++] = (( uint16_t)temp_voltage>> 8) & 0xFF;
	buf[index++] = (((uint16_t)temp_voltage)) & 0xFF;
	buf[index++] = (((uint16_t)temp_batteryPercentage)) & 0xFF;
	buf[index++] = (( uint16_t)temp_electricity>> 8) & 0xFF;
	buf[index++] = (((uint16_t)temp_electricity)) & 0xFF;
	buf[index++] = (( uint16_t)temp1_m_tmp>> 8) & 0xFF;
	buf[index++] = (((uint16_t)temp1_m_tmp)) & 0xFF;
	buf[index++] = (( uint16_t)temp2_m_tmp>> 8) & 0xFF;
	buf[index++] = (((uint16_t)temp2_m_tmp)) & 0xFF;
	buf[index++] = (( uint16_t)temp3_m_tmp>> 8) & 0xFF;
	buf[index++] = (((uint16_t)temp3_m_tmp)) & 0xFF;
	buf[2] = index-3;//len
	byte crc = 0;
	for(int i = 2; i < index; i++)
		crc ^= buf[i];
	buf[index++] = crc;
	mySerialWriteUSART4(buf,index);
	mySerialWriteUSART3(buf,index);
}
void COBD_loop(CODB_parameter* CODB_para)
{
	if( CODB_para->m_DS18B20.DS18B20_Num >= 1 )//判断 是否有 温度传感器
		COBD_getTemp(CODB_para);

	if( millis() - CODB_para->m_volGetTime >= 100 ){
		int detCODB=millis() - CODB_para->m_volGetTime;
		CODB_para->m_volGetTime = millis();
		CODB_para->m_volReadCount++;
		CODB_para->m_voltage += COBD_getVol(detCODB);
	}

	if( millis() - CODB_para->m_systemTime > 1000 ){//5s输出一次obd信息
		char strOutput[70]="";
		float temp_voltage,temp_electricity,temp1_m_tmp,temp2_m_tmp,temp3_m_tmp;
		int temp_batteryPercentage;
		COBD_getElectricity(CODB_para);
		CODB_para->m_voltage /= (float)CODB_para->m_volReadCount;//计算每100ms电池电压采样的计数平均值
		CODB_para->m_stableVol = CODB_para->m_voltage;
		if( CODB_para->m_getTimpReady ){
			CODB_para->m_getTimpReady = false;
			for( int i =0; i < CODB_para->m_DS18B20.m_sensorNum; i++ )
					add_num_to_str_float( strOutput, CODB_para->m_tmp[i] );
			}
			temp_voltage=(CODB_para->m_voltage)*100;
			temp_batteryPercentage=CODB_para->m_batteryPercentage;
			//temp_batteryPercentage = COBD_getBatteryPercentage(CODB_para->m_voltage);
			temp_electricity=(CODB_para->m_electricity)*100;
			temp1_m_tmp=(CODB_para->m_tmp[0])*100;
			temp2_m_tmp=(CODB_para->m_tmp[1])*100;
			temp3_m_tmp=(CODB_para->m_tmp[2])*100;
			//发送COBD状态
			COBD_printfEncoding((uint16_t)temp_voltage,(uint16_t)temp_batteryPercentage,(uint16_t)temp_electricity
								,(uint16_t)temp1_m_tmp,(uint16_t)temp2_m_tmp,(uint16_t)temp3_m_tmp);
			CODB_para->m_systemTime = millis();
			CODB_para->m_volReadCount = 0;
			CODB_para->m_voltage = 0;
	}
}

void EXTI2_IRQHandler(void)
{
	if( EXTI_GetFlagStatus(EXTI_Line2) == !RESET ){
		EXTI_ClearITPendingBit(EXTI_Line2);
		m_volCount++;
		return;
	}
	EXTI_ClearITPendingBit(EXTI_Line2);
	
}

float COBD_getStableVol(CODB_parameter* CODB_para)
{
	return CODB_para->m_stableVol;
}
