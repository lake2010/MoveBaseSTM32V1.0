#ifndef _I2C_H
#define _I2C_H

#include "config.h"
#define RT_USING_I2C_BITOPS
#define RT_I2C_BIT_DEBUG

//#define _debug_i2c
#ifdef  _debug_i2c
#define i2c_printf	myprintf
#else
#define i2c_printf
#endif

void I2C1_Init( void );

void I2C_readByte(	I2C_TypeDef* I2Cx,
					uint8_t slave_address,
					uint8_t readAddr,
					uint8_t *data);
void I2C_writeByte(	I2C_TypeDef* I2Cx,
					uint8_t slave_address,
					uint8_t writeAddr,
					uint8_t data);
void I2C_readBytes(	I2C_TypeDef* I2Cx,
					uint8_t slave_address,
					uint8_t readAddr,
					uint8_t length,
					uint8_t *data);
void I2C_writeBytes(I2C_TypeDef* I2Cx,
					uint8_t slave_address,
					uint8_t writeAddr,
					uint8_t length,
					uint8_t *data);
void I2C_readBit(	I2C_TypeDef* I2Cx,
					uint8_t slave_address,
					uint8_t regAddr,
					uint8_t bitNum,
					uint8_t *data);
void I2C_readBits(	I2C_TypeDef* I2Cx,
					uint8_t slave_address,
					uint8_t regAddr,
					uint8_t bitStart,
					uint8_t length,
					uint8_t *data);
void I2C_writeBit(	I2C_TypeDef* I2Cx,
					uint8_t slave_address,
					uint8_t regAddr,
					uint8_t bitNum,
					uint8_t data);
void I2C_writeBits(	I2C_TypeDef* I2Cx,
					uint8_t slave_address,
					uint8_t regAddr,
					uint8_t bitStart,
					uint8_t length,
					uint8_t data);
void I2C_writeWord(	I2C_TypeDef* I2Cx,
					uint8_t slave_address,
					uint8_t writeAddr,
					uint16_t data);

#endif
