#include "stm32f10x.h"                  // Device header
#include "stm32f10x_flash.h"            // Device:StdPeriph Drivers:Flash

#define  ADDR_STR_BLINK 0x08008000 


void Boot(){
	RCC_DeInit();
	SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk| SCB_SHCSR_BUSFAULTENA_Msk| SCB_SHCSR_MEMFAULTENA_Msk);
	
	__set_MSP(*(__IO uint32_t*)(ADDR_STR_BLINK));
	
	uint32_t JumpAddress = *(__IO uint32_t*)(ADDR_STR_BLINK+4);
	
	void (*reset_handler)(void) = (void *)JumpAddress;
	reset_handler();
}




int main()
{
		Boot();
		while(1)
			{
				
				}
}