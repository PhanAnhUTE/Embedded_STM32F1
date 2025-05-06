#include "stm32f10x.h"                  // Device header
#include "stm32f10x_exti.h"             // Device:StdPeriph Drivers:EXTI
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC


void RCC_Config()
{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		
}


void GPIO_Config(){
GPIO_InitTypeDef GPIOInitStruct;
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIOInitStruct);
	//
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIOInitStruct);
}


void UART_Config(){
	USART_InitTypeDef UART_InitStruct;
	UART_InitStruct.USART_Mode = USART_Mode_Rx| USART_Mode_Tx;
	UART_InitStruct.USART_BaudRate = 9600;
	UART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART_InitStruct.USART_WordLength = USART_WordLength_8b;
	UART_InitStruct.USART_StopBits = USART_StopBits_1;
	UART_InitStruct.USART_Parity = USART_Parity_No;
	USART_Init(USART1, &UART_InitStruct);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	
	USART_Cmd(USART1, ENABLE);
}

void NVIC_Config()
{
		NVIC_InitTypeDef NVIC_InitStruct;

		NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	
		NVIC_Init(&NVIC_InitStruct);

}


void USART1_IRQHandler(){
	uint8_t data = 0x00;
	if(USART_GetITStatus(USART1, USART_IT_RXNE)!=RESET){
		data = USART_ReceiveData(USART1);
		while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE));
		if(USART_GetITStatus(USART1, USART_IT_TXE)==RESET){
			USART_SendData(USART1, data);
		}
	}
	USART_ClearITPendingBit (USART1, USART_IT_RXNE);
}




int main()
{
		RCC_Config();
		GPIO_Config();
		NVIC_Config();
		UART_Config();
		
	
		while(1)
			{
					
				}
		
}


