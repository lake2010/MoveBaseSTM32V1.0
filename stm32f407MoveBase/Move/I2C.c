#include "I2C.h"

#define MAX_TIMEOUTCOUNT 1100

void I2C1_Init( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
	
	I2C_DeInit(I2C1);
    I2C_InitStructure.I2C_ClockSpeed = 100000; 
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
}

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
	int timeOut = 0;
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)){
		timeOut++;
		if( timeOut > MAX_TIMEOUTCOUNT ){
			//i2c_printf("i2c TimeOut start\r\n");
			return;
		}
	} 
	I2C_GenerateSTART(I2Cx, ENABLE);
	timeOut = 0;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		timeOut++;
		if( timeOut > MAX_TIMEOUTCOUNT ){
			//i2c_printf("i2c TimeOut start em\r\n");
			return;
		}
	} 
	I2C_Send7bitAddress(I2Cx, address<<1, direction);

	if(direction == I2C_Direction_Transmitter){
		timeOut = 0;
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
			timeOut++;
			if( timeOut > MAX_TIMEOUTCOUNT ){
				//i2c_printf("i2c TimeOut tr\r\n");
				return;
		}
		} 
	}
	else if(direction == I2C_Direction_Receiver){
		timeOut = 0;
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
			timeOut++;
			if( timeOut > MAX_TIMEOUTCOUNT ){
				//i2c_printf("i2c TimeOut re\r\n");
				return;
			}
		} 
	}
}

void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	int timeOut = 0;
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		timeOut++;
		if( timeOut > MAX_TIMEOUTCOUNT ){
			//i2c_printf("i2c TimeOut wr\r\n");
			return;
		}
	} 
}

uint8_t I2C_read_ack(I2C_TypeDef* I2Cx)
{
	uint8_t data;
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	int timeOut = 0;
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ){
		timeOut++;
		if( timeOut > MAX_TIMEOUTCOUNT ){
			//i2c_printf("i2c TimeOut rea\r\n");
			return 0;
		}
	} 
	data = I2C_ReceiveData(I2Cx);
	return data;
}

uint8_t I2C_read_nack(I2C_TypeDef* I2Cx)
{
	uint8_t data; 
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	int timeOut = 0;
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ){
		timeOut++;
		if( timeOut > MAX_TIMEOUTCOUNT ){
			//i2c_printf("i2c TimeOut rena\r\n");
			return 0;
		}
	} 
	data = I2C_ReceiveData(I2Cx);
	return data;
}

void I2C_stop(I2C_TypeDef* I2Cx)
{

	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void I2C_readByte(I2C_TypeDef* I2Cx, uint8_t slave_address, uint8_t readAddr, uint8_t *data)
{
	
	I2C_start(I2Cx, slave_address, I2C_Direction_Transmitter);
	I2C_write(I2Cx, readAddr); 
	I2C_stop(I2Cx);
	I2C_start(I2Cx, slave_address, I2C_Direction_Receiver); 
	*data = I2C_read_nack(I2Cx); 
}

void I2C_writeByte(I2C_TypeDef* I2Cx, uint8_t slave_address, uint8_t writeAddr, uint8_t data)
{

	I2C_start(I2Cx, slave_address, I2C_Direction_Transmitter); 
	I2C_write(I2Cx, writeAddr);
	I2C_write(I2Cx, data);
	I2C_stop(I2Cx);
}

void I2C_readBytes(I2C_TypeDef* I2Cx, uint8_t slave_address, uint8_t readAddr, uint8_t length, uint8_t *data)
{

	I2C_start(I2Cx, slave_address, I2C_Direction_Transmitter); 	
	I2C_write(I2Cx, readAddr); 																	
	I2C_stop(I2Cx); 																						
	I2C_start(I2Cx, slave_address, I2C_Direction_Receiver); 		
	while(length){
		if(length==1)
			*data = I2C_read_nack(I2Cx);
		else
			*data = I2C_read_ack(I2Cx);																

		data++;
		length--;
	}
}

void I2C_writeBytes(I2C_TypeDef* I2Cx, uint8_t slave_address, uint8_t writeAddr, uint8_t length, uint8_t *data){

	int i=0;
	I2C_start(I2Cx, slave_address, I2C_Direction_Transmitter);
	I2C_write(I2Cx, writeAddr);
	for(i=0; i<length; i++){	
		I2C_write(I2Cx, data[i]);
	}
	I2C_stop(I2Cx);
}

void I2C_readBit(I2C_TypeDef* I2Cx, uint8_t slave_address, uint8_t regAddr, uint8_t bitNum, uint8_t *data){

	uint8_t tmp;
	I2C_readByte(I2Cx, slave_address, regAddr, &tmp);  
	*data = tmp & (1 << bitNum);
}

void I2C_readBits(I2C_TypeDef* I2Cx, uint8_t slave_address, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
	uint8_t mask,tmp;
	I2C_readByte(I2Cx, slave_address, regAddr, &tmp); 
	mask = ((1 << length) - 1) << (bitStart - length + 1);
	tmp &= mask;
	tmp >>= (bitStart - length + 1);
	*data = tmp;
}

void I2C_writeBit(I2C_TypeDef* I2Cx, uint8_t slave_address, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{

	uint8_t tmp;
	I2C_readByte(I2Cx, slave_address, regAddr, &tmp);  
	tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
	I2C_writeByte(I2Cx, slave_address,regAddr,tmp); 
}

void I2C_writeBits(I2C_TypeDef* I2Cx, uint8_t slave_address, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
	uint8_t tmp,mask;
	I2C_readByte(I2Cx, slave_address, regAddr, &tmp);
	mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1); 
	data &= mask; 
	tmp &= ~(mask); 
	tmp |= data; 
	I2C_writeByte(I2Cx, slave_address, regAddr, tmp);	
}

void I2C_writeWord(I2C_TypeDef* I2Cx, uint8_t slave_address, uint8_t writeAddr, uint16_t data)
{
	I2C_start(I2Cx, slave_address, I2C_Direction_Transmitter); 
	I2C_write(I2Cx, writeAddr);         
	I2C_write(I2Cx, (data >> 8));      // send MSB
	I2C_write(I2Cx, (data << 8));			 // send LSB
	I2C_stop(I2Cx);
}

