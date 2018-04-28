#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "config.h"

#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH first address
 

//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 	//flash 0 , 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000) 	//flash 1 , 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000) 	//flash 2 , 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 	//flash 3 , 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 	//flash 4 , 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 	//flash 5 , 128 Kbytes  
//#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	//flash 6 , 128 Kbytes  
//#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000) 	//flash 7 , 128 Kbytes  
//#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000) 	//flash 8 , 128 Kbytes  
//#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	//flash 9 , 128 Kbytes  
//#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 	//flash 10 ,128 Kbytes  
//#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 	//flash 11 ,128 Kbytes  

#define ParamBuffSize sizeof(DW1000Param_buff)
#define ParamBuffLen  ParamBuffSize/4+((ParamBuffSize%4)?1:0)
// 0--shortAddress
// 1--tagNum
// 2--tagID
extern u32 DW1000Param_buff[4];

u32 STMFLASH_ReadWord(u32 faddr);
int STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite);
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead);

#endif
