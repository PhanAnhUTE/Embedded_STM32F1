#include "stm32f10x.h"                  // Device header
#include "stm32f10x_flash.h"            // Device:StdPeriph Drivers:Flash


void Flash_Erase(uint32_t addresspage){
	FLASH_Unlock();
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	FLASH_ErasePage(addresspage);
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	FLASH_Lock();
}


void Flash_WriteInt(uint32_t address, uint16_t value){
	FLASH_Unlock();
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	FLASH_ProgramHalfWord(address, value);
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	FLASH_Lock();
}


void Flash_WriteNumByte(uint32_t address, uint8_t *data, int num){
		
	FLASH_Unlock();
	while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
	uint16_t *ptr = (uint16_t*)data;
	for(int i=0; i<((num+1)/2); i++){
		FLASH_ProgramHalfWord(address+2*i, *ptr);
		while(FLASH_GetFlagStatus(FLASH_FLAG_BSY) == 1);
		ptr++;
	}
	FLASH_Lock();
}




uint8_t data[14] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14};
int main()
{
				Flash_Erase(0x0800F000);
				Flash_WriteNumByte(0x0800F000, data, 13);
		while(1){
				
			}
}