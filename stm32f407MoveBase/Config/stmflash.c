#include "stmflash.h"

u32 DW1000Param_buff[4];

u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32*)faddr; 
}  

uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if( addr < ADDR_FLASH_SECTOR_1 )
		return FLASH_Sector_0;
	else if( addr < ADDR_FLASH_SECTOR_2 )
		return FLASH_Sector_1;
	else if( addr < ADDR_FLASH_SECTOR_3 )
		return FLASH_Sector_2;
	else if( addr < ADDR_FLASH_SECTOR_4 )
		return FLASH_Sector_3;
	else if( addr >= ADDR_FLASH_SECTOR_4 && addr < ADDR_FLASH_SECTOR_5 )
		return FLASH_Sector_4;
	return FLASH_Sector_11;	
}

int STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{
	
	FLASH_Status status = FLASH_COMPLETE;
	u32 addrx=0;
	u32 endaddr=0;	
	if( WriteAddr < STM32_FLASH_BASE || WriteAddr % 4 ){
		return 0;
	}
	FLASH_Unlock();
	FLASH_DataCacheCmd(DISABLE);//enable dataCache
		
	addrx=WriteAddr;
	endaddr=WriteAddr + NumToWrite * 4;
	if( addrx < 0X1FFF0000 )
	{
		while( addrx < endaddr )
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)
			{   
				status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);
				if(status!=FLASH_COMPLETE){
					myprintf("Flash write err!\r\n");
					break;	//error
				}
			}else addrx+=4;
		} 
	}
	if(status==FLASH_COMPLETE)
	{
		while(WriteAddr<endaddr)//write data
		{
			if(FLASH_ProgramWord(WriteAddr,*pBuffer)!=FLASH_COMPLETE){ 
				myprintf("Flash write err!\r\n");
				break;	//erite err
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
	FLASH_DataCacheCmd(ENABLE);	//ENABLE datacache
	FLASH_Lock();//
	return 1;
} 

void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++){
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);
		ReadAddr+=4;
	}
}














