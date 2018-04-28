#include "spi.h"
#include "stm32f4xx.h"
//#include "DW1000.h"
/*配置SPI1为双向全双工，主机模式，8bit，时钟 4分频21M*/
void SPI1_Init(void) 
{
	/*设置SPI及对应I/O*/
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
    /*开启GPIOA时钟*/
	//GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//GPIOC
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	//设置SPI1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 , ENABLE);
	
	//初始化SPI NSS片选引脚 PA4为普通I/O输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//LED0和LED1对应IO口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA4
	
	//初始化SPI IO 使用PA5 PA6 PA7
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		          
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//使用推免输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource4,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
	
	/*SPI口初始化*/
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1
	
	//SPI模式设置
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//传输模式此为2线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//主从模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//数据位数 8位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//空闲时clk电平状态
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//0奇跳变采样 1偶跳变采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS片选设置，这里是软件片选
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//速度
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//数据发送的是从低位还是高位
	SPI_InitStructure.SPI_CRCPolynomial = 7;//校验设置
	SPI_Init(SPI1, &SPI_InitStructure);
	//使能SPI
	SPI_Cmd(SPI1, ENABLE);
	
	SPI1_CS_High;   //片选置高
}

void SPI1_FastSpeed(void)
{
	//SPI模式设置
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//传输模式此为2线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//主从模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//数据位数 8位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//空闲时clk电平状态
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//0奇跳变采样 1偶跳变采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS片选设置，这里是软件片选
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//速度
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//数据发送的是从低位还是高位
	SPI_InitStructure.SPI_CRCPolynomial = 7;//校验设置
	SPI_Init(SPI1, &SPI_InitStructure);
	//使能SPI
	SPI_Cmd(SPI1, ENABLE);
	
	SPI1_CS_High;   //片选置高
}

void SPI1_End(void)
{
	SPI1_CS_High;   //片选置高
}

/*
通过SPI发送，并且接受到一个字节的数据
data：发送数据的存放指针
返回值：读取的数据
*/
u8 SPI_send_and_receive_byte(u8 data) {
	
	u8 retry = 0;
	
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){
		retry++;
		if( retry > 200 ){
			myprintf("send error\r\n");
			return 0;
		}
	}
	//myprintf( "T\r\n" );
	SPI_I2S_SendData(SPI1, data);
	retry = 0;
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){
		retry++;
		if( retry > 200 ){
			myprintf("receive error\r\n");
			return 0;
		}
	}
	//myprintf( "S\r\n" );
	return SPI_I2S_ReceiveData(SPI1);
}

