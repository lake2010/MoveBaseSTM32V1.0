#include "DS18B20.h"

void DS18B20_searchRom( DS18B20_parameter* DS18B20_para );
void DS18B20_writeByte( DS18B20_parameter* DS18B20_para, u8 dat );
void DS18B20_writeBit( DS18B20_parameter* DS18B20_para, u8 dat );
u8 DS18B20_readByte( DS18B20_parameter* DS18B20_para );
u8 DS18B20_read2Bit( DS18B20_parameter* DS18B20_para );
u8 DS18B20_readBit( DS18B20_parameter* DS18B20_para );
u8 DS18B20_answerCheck( DS18B20_parameter* DS18B20_para );
void DS18B20_Reset( DS18B20_parameter* DS18B20_para );
void DS18B20_setPinOutPut( DS18B20_parameter* DS18B20_para );
void DS18B20_setPinInPut( DS18B20_parameter* DS18B20_para );

u8 DS18B20_setup( DS18B20_parameter*	DS18B20_para, 
					Gpio_pin_parameter*	Gpio_pin, 
					unsigned char		SensorNum)
{
	DS18B20_para->m_busPin = Gpio_pin;
	pinOutputModeInit(DS18B20_para->m_busPin);//设置引脚功能
	GPIO_SetBits(DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin);
	DS18B20_Reset(DS18B20_para);  
	return DS18B20_answerCheck(DS18B20_para);  
}

//引脚设置为输入上拉
void DS18B20_setPinInPut( DS18B20_parameter* DS18B20_para )
{
	GPIO_InitTypeDef GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Pin = DS18B20_para->m_busPin->m_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(DS18B20_para->m_busPin->m_gpio, &GPIO_InitStructure);
}

//引脚设置为输出推挽
void DS18B20_setPinOutPut( DS18B20_parameter* DS18B20_para )
{
	pinOutputModeInit(DS18B20_para->m_busPin);
}

//复位，主机给从机发送复位信号
void DS18B20_Reset( DS18B20_parameter* DS18B20_para )
{
	DS18B20_setPinOutPut( DS18B20_para );
	DS18B20_busPinOutLow( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );
	delay_us(480);
	DS18B20_busPinOutHigh( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );
	delay_us(15);
}

// 检测从机给主机返回的应答脉冲。从机接收到主机的复位信号后，会在15~60us后给主机发一个应答脉冲  
u8 DS18B20_answerCheck( DS18B20_parameter* DS18B20_para )  
{  
	u8 delay = 0;  
	DS18B20_setPinInPut( DS18B20_para ); // 主机设置为上拉输入  
	// 等待应答脉冲（一个60~240us的低电平信号）的到来  
	// 如果100us内，没有应答脉冲，退出函数，注意：从机接收到主机的复位信号后，会在15~60us后给主机发一个存在脉冲  
	while(	DS18B20_busPinIn( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin ) &&
			delay < 100 ){
				delay++;  
				delay_us(1);  
	}  
	// 经过100us后，如果没有应答脉冲，退出函数  
	if (delay >= 100)//Hu200
		return 1;  
	else  
		delay = 0;  
	// 有应答脉冲，且存在时间不超过240us  
	while(	!DS18B20_busPinIn( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin ) &&
			delay < 240){
				delay++;  
				delay_us(1);
	}  
	if (delay >= 240)  
		return 1;  
	return 0;  
}

// 从DS18B20读取1个位  
u8 DS18B20_readBit( DS18B20_parameter* DS18B20_para )  
{  
	u8 data;  
	DS18B20_setPinOutPut( DS18B20_para );
	DS18B20_busPinOutLow( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );// 读时间的起始：必须由主机产生 >1us <15us 的低电平信号  
	delay_us(2);  
	DS18B20_busPinOutHigh( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );
	delay_us(12);  
	DS18B20_setPinInPut( DS18B20_para );// 设置成输入，释放总线，由外部上拉电阻将总线拉高  
	if (DS18B20_busPinIn( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin ))  
		data = 1;  
	else  
		data = 0;  
	delay_us(50);  
	return data;
}

// 从DS18B20读取2个位  
u8 DS18B20_read2Bit( DS18B20_parameter* DS18B20_para )//读二位 子程序  
{  
	u8 i;  
	u8 dat = 0;  
	for (i = 2; i > 0; i--){
		dat = dat << 1;  
		DS18B20_setPinOutPut( DS18B20_para );  
		DS18B20_busPinOutLow( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );  
		delay_us(2);  
		DS18B20_busPinOutHigh( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );  
		DS18B20_setPinInPut(DS18B20_para);  
		delay_us(12);  
		if (DS18B20_busPinIn(DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin))  
			dat |= 0x01;  
		delay_us(50);  
	}  
	return dat;  
}

// 从DS18B20读取1个字节  
u8 DS18B20_readByte( DS18B20_parameter* DS18B20_para )  // read one byte  
{  
	u8 i, j, dat;  
	dat = 0;  
	for (i = 0; i < 8; i++){  
		j = DS18B20_readBit( DS18B20_para );  
		dat = (dat) | (j << i);  
	}  
	return dat;  
}

// 写1位到DS18B20  
void DS18B20_writeBit( DS18B20_parameter* DS18B20_para, u8 dat )
{  
	DS18B20_setPinOutPut( DS18B20_para );  
	if (dat){  
		DS18B20_busPinOutLow( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );// Write 1  
		delay_us(2);  
		DS18B20_busPinOutHigh( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );  
		delay_us(60);  
	}else{  
		DS18B20_busPinOutLow( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );// Write 0  
		delay_us(60);  
		DS18B20_busPinOutHigh( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );  
		delay_us(2);  
	}  
}

// 写1字节到DS18B20  
void DS18B20_writeByte( DS18B20_parameter* DS18B20_para, u8 dat )
{  
	u8 j;  
	u8 testb;  
	DS18B20_setPinOutPut( DS18B20_para );  
	for (j = 1; j <= 8; j++){  
		testb = dat & 0x01;  
		dat = dat >> 1;  
		if (testb){  
			DS18B20_busPinOutLow( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );// 写1  
			delay_us(10);  
			DS18B20_busPinOutHigh( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );  
			delay_us(50);  
		}else{  
			DS18B20_busPinOutLow( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );// 写0  
			delay_us(60);  
			DS18B20_busPinOutHigh( DS18B20_para->m_busPin->m_gpio, DS18B20_para->m_busPin->m_pin );// 释放总线  
			delay_us(2);  
		}  
	}  
}

// 从ds18b20得到温度值，精度：0.1C，返回温度值（-550~1250），Temperature1返回浮点实际温度  
float DS18B20_getTemp( DS18B20_parameter* DS18B20_para, u8 i)
{  
	//u8 flag;  
	u8 j;//匹配的字节  
	u8 TL, TH;  
	short Temperature;  
	float Temperature1;  
	DS18B20_Reset(DS18B20_para);  
	DS18B20_answerCheck(DS18B20_para);  
	DS18B20_writeByte(DS18B20_para, 0xcc);// skip rom  
	DS18B20_writeByte(DS18B20_para, 0x44);// convert  
	DS18B20_Reset(DS18B20_para);  
	DS18B20_answerCheck(DS18B20_para);  

	// DS18B20_Write_Byte(0xcc);// skip rom  
	//匹配ID，i为形参  
	DS18B20_writeByte(DS18B20_para, 0x55);  
	for (j = 0; j < 8; j++){  
		DS18B20_writeByte(DS18B20_para, DS18B20_para->DS18B20_ID[i][j]);  
	}  

	DS18B20_writeByte(DS18B20_para, 0xbe);// convert  
	TL = DS18B20_readByte(DS18B20_para); // LSB     
	TH = DS18B20_readByte(DS18B20_para); // MSB    
	if (TH & 0xfc){  
		//flag=1;  
		Temperature = (TH << 8) | TL;  
		Temperature1 = (~Temperature) + 1;  
		Temperature1 *= (float)0.0625;  
	}  
	else{  
		//flag=0;  
		Temperature1 = ((TH << 8) | TL)*0.0625;  
	}  
	return Temperature1;  
}

// 自动搜索ROM  
void DS18B20_searchRom( DS18B20_parameter* DS18B20_para )
{  
	u8 k, l, chongtuwei, m, n, num;  
	u8 zhan[5];  
	u8 ss[64];  
	u8 tempp;  
	l = 0;  
	num = 0;  
	do{  
		DS18B20_Reset(DS18B20_para); //注意：复位的延时不够  
		delay_us(480); //480、720  
		DS18B20_writeByte(DS18B20_para, 0xf0);  
		for (m = 0; m < 8; m++){  
			u8 s = 0;  
			for (n = 0; n < 8; n++){  
				k = DS18B20_read2Bit(DS18B20_para);//读两位数据  

				k = k & 0x03;  
				s >>= 1;  
				if (k == 0x01){  
					DS18B20_writeBit(DS18B20_para, 0);  
					ss[(m * 8 + n)] = 0;  
				}else if (k == 0x02){  //读到的数据为1 写1 此位为1的器件响应
					s = s | 0x80;  
					DS18B20_writeBit(DS18B20_para, 1);  
					ss[(m * 8 + n)] = 1;  
				}else if (k == 0x00){  //读到的数据为00 有冲突位 判断冲突位 
					//如果冲突位大于栈顶写0 小于栈顶写以前数据 等于栈顶写1  
					chongtuwei = m * 8 + n + 1;  
					if (chongtuwei > zhan[l]){  
						DS18B20_writeBit(DS18B20_para, 0);  
						ss[(m * 8 + n)] = 0;  
						zhan[++l] = chongtuwei;  
					}else if (chongtuwei < zhan[l]){  
						s = s | ((ss[(m * 8 + n)] & 0x01) << 7);  
						DS18B20_writeBit(DS18B20_para, ss[(m * 8 + n)]);  
					}else if (chongtuwei == zhan[l]){  
						s = s | 0x80;  
						DS18B20_writeBit(DS18B20_para, 1);  
						ss[(m * 8 + n)] = 1;  
						l = l - 1;  
					}  
				}else{  
					//没有搜索到  
				}  
			}  
			tempp = s;  
			DS18B20_para->DS18B20_ID[num][m] = tempp; // 保存搜索到的ID  
		}  
		num = num + 1;// 保存搜索到的个数  
	} while (zhan[l] != 0 && (num < DS18B20_para->m_sensorNum));  
	DS18B20_para->DS18B20_Num = num;  
	//printf("DS18B20_SensorNum=%d\r\n",DS18B20_SensorNum);  
}
