#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_usart.h"            // Device:StdPeriph Drivers:USART
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM


#define TX_Pin GPIO_Pin_9
#define RX_Pin GPIO_Pin_10
#define UART_GPIO GPIOA




void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}


void UART_Config(){
		
			USART_InitTypeDef UART_InitStruct;
			UART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
			UART_InitStruct.USART_BaudRate = 9600;
			UART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			UART_InitStruct.USART_WordLength = USART_WordLength_8b;
			UART_InitStruct.USART_StopBits = USART_StopBits_1;
			UART_InitStruct.USART_Parity = USART_Parity_No;
	
			USART_Init(USART1, &UART_InitStruct);
			USART_Cmd(USART1,ENABLE);
	
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




char UART_ReceiveChar(USART_TypeDef *USARTx)
	{
			USART1 -> DR = 0x00;
			char tmp = 0x00;
			tmp = 0x00;
			tmp = USART_ReceiveData(USARTx);
		
			while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET);
		
			return tmp; 
		}

		
		
void UART_SendChar(USART_TypeDef *USARTx, char data)
{
    USARTx->DR = 0x00;
    USART_SendData(USARTx, data);

    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
}



void UART_SendString(USART_TypeDef *USARTx, char *str)
{
    while (*str)
    {
        UART_SendChar(USARTx, *str);
        str++;
    }
}


char data = 'a';

int main()
{
    GPIO_Config();
		RCC_Config();
    UART_Config();

    while (1)
    {
        data = UART_ReceiveChar(USART1);
        UART_SendChar(USART1, data);
    }
}






