#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"


void delay(uint32_t time)
	{	
			for(int i = 0; i< time; i++){}
		}

void RCC_Config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}
void GPIO_Config()
{
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC, &GPIO_InitStruct);
}

int main()
	{
		RCC_Config();
		GPIO_Config();
		
		while(1)
			{
					GPIO_ResetBits(GPIOC, GPIO_Pin_13);
					delay(1000000);
					GPIO_SetBits(GPIOC, GPIO_Pin_13);
					delay(1000000);

					
				}
		}