/*KS10X.c*/
#include "I2C_Init.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "math.h"
#include "ks103.h"

#define uchar unsigned char
#define uint unsigned int

void Single_WriteI2C(u8 SlaveAddress, u8 REG_Address, u8 REG_data)
{
	IIC_Start();                    //起始信号

	IIC_Send_Byte(SlaveAddress);   //发送设备地址+写信号
	//while(IIC_Wait_Ack());       //有问题，不能用while等待
	IIC_Wait_Ack();

	IIC_Send_Byte(REG_Address);    //内部寄存器地址
	//while(IIC_Wait_Ack());       //有问题，不能用while等待
	IIC_Wait_Ack();

	IIC_Send_Byte(REG_data);       //内部寄存器数据
	//while(IIC_Wait_Ack());       //有问题，不能用while等待
	IIC_Wait_Ack();

	IIC_Stop();                     //发送停止信号
}

//**************************************
//从I2C设备读取一个字节数据
//**************************************
u8 Single_ReadI2C(u8 SlaveAddress, u8 REG_Address)
{
	u8 REG_data = 0;

	IIC_Start();

	IIC_Send_Byte(SlaveAddress);
	//while(IIC_Wait_Ack());       //有问题，不能用while等待
	IIC_Wait_Ack();

	IIC_Send_Byte(REG_Address);
	//while(IIC_Wait_Ack());      //有问题，不能用while等待
	IIC_Wait_Ack();

	IIC_Start();

	IIC_Send_Byte(SlaveAddress + 1);
	//while(IIC_Wait_Ack());      //有问题，不能用while等待
	IIC_Wait_Ack();

	REG_data = IIC_Read_Byte(0);

	IIC_Stop();

	return REG_data;
}

float Read_KS10X_Data(u8 SlaveAddress, u8 command)
{
	u16 i = 0;
	u16 sumx = 0.0;
	float sum_x = 0.0;

	Single_WriteI2C(SlaveAddress, 0x02, command);

	if (command == 0xb4 || command == 0xbc || command == 0xbf)
		delay_ms(90);
	else if (command == 0xb8 || command == 0xba || command == 0xbd || command == 0xbe)
		delay_ms(70);
	else if (command == 0xb0 || command == 0xb2)
		delay_ms(80);
	else if (command == 0xa0)
		delay_ms(1);

	sumx = Single_ReadI2C(SlaveAddress, 0x02);        //高8位
	sumx <<= 8;
	sumx += Single_ReadI2C(SlaveAddress, 0x03);       //低8位

	sum_x = (float)sumx / 10;          //mm转换为cm

	return sum_x;
}

void KS10X_Change_Addr_Init(u8 OldAddr, u8 NewAddr)  //此函数用来实现选择超声波的地址
{
	Change_Addr(OldAddr, NewAddr);
	delay_ms(80);
}

void Change_Addr(u8 OldAdddr, u8 NewAddr)
{
	Single_WriteI2C(OldAdddr, 0x02, 0x9a);             //默认原地址是0x00;
	delay_ms(1);

	Single_WriteI2C(OldAdddr, 0x02, 0x92);
	delay_ms(1);

	Single_WriteI2C(OldAdddr, 0x02, 0x9e);
	delay_ms(1);

	Single_WriteI2C(OldAdddr, 0x02, NewAddr);
	delay_ms(100);

	Single_WriteI2C(NewAddr, 0x02, 0xb0);
	delay_ms(80);
}

	/*
	delay_ms(1);
	SCL_IN();
	for (i = 0; i < 100; i++)
	{
		SCL_IN();
		if (READ_SCL == 0)
			printf("wait %d for detect finish.\r\n", i);
		else
		{
			printf("detect finish\r\n");
			break;
		}
		delay_ms(1);
	}
	SCL_OUT();
	*/
