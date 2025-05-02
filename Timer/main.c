#include "stm32f10x.h"
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM

void delay_ms(uint8_t timedelay)
	{
			TIM_SetCounter(TIM2,0);
			while(TIM_GetCounter(TIM2)<timedelay*10){}
		}

void RCC_Config()
	{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		}

void TIM_Config()
	{
			TIM_TimeBaseInitTypeDef TIM_InitStruct;
			
			TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
			//clock timer = 72M/1 = 72MHZ
			TIM_InitStruct.TIM_Prescaler = 7200 - 1; // 1ms
			TIM_InitStruct.TIM_Period = 0xffff;
			TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
		
			TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
			
			TIM_Cmd(TIM2, ENABLE);
		}
		
		
int main()
	{

		}
	