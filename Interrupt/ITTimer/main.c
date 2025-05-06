#include "stm32f10x.h"                  // Device header
#include "stm32f10x_exti.h"             // Device:StdPeriph Drivers:EXTI
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM


void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}


void TIM_Config(){
   
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		TIM_TimeBaseInitStruct.TIM_Prescaler = 7200-1;
		TIM_TimeBaseInitStruct.TIM_Period = 10000-1;
		TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void NVIC_Config()
	{
			NVIC_InitTypeDef NVIC_InitStruct;

			NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
			NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
			NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStruct);


		}



/*uint16_t count;
void delay_ms(int time){
		count=0; 
		while(count<time){}
} */

					
uint16_t count = 0;
void TIM2_IRQHandler()
{		
		if(TIM_GetITStatus(TIM2, TIM_IT_Update)){
							
			count++;
						// Clears the TIM2 interrupt pending bit
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);}
}

int main()
	{
					RCC_Config();
					TIM_Config();
					NVIC_Config();
			while(1){
					
				}
		}

