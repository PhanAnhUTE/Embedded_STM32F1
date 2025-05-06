#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_exti.h"             // Device:StdPeriph Drivers:EXTI
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO


void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}


void GPIO_Config(){
GPIO_InitTypeDef GPIOInitStruct;
	//
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIOInitStruct);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);
	
}

void EXTI_Config()
	{
			EXTI_InitTypeDef EXTI_InitStruct;
			EXTI_InitStruct.EXTI_Line = EXTI_Line0;
			EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
			EXTI_InitStruct.EXTI_LineCmd = ENABLE;

			EXTI_Init(&EXTI_InitStruct);
		
		}
	

void NVIC_Config()
	{
			NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		
			NVIC_InitTypeDef NVIC_InitStruct;
			NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
			NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
			NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
			
			NVIC_Init(&NVIC_InitStruct);
		
		}
		
void EXTI0_IRQHandler(void)
	{
			if(EXTI_GetITStatus(EXTI_Line0) != RESET)
				{
						// GPIOC->ODR ^= (1<<13);
					}
				
			 EXTI_ClearITPendingBit(EXTI_Line0);
		}
		
int main()
	{
			
		}