#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM

#define TX_Pin GPIO_Pin_9
#define RX_Pin GPIO_Pin_10
#define UART_GPIO GPIOA
#define BRateTime 105


typedef enum{
	Parity_Mode_NONE,
	Parity_Mode_ODD,
	Parity_Mode_EVENT
}Parity_Mode;



uint8_t Parity_Generate(uint8_t data, Parity_Mode Mode){
	uint8_t count =0;
	uint8_t data1 = data;
	for(int i=0; i< 8; i++){
		if(data1 & 0x01){
			count++;
		}
		data1>>=1;
	}
	switch(Mode){
		case Parity_Mode_NONE:
			return data; 
			break;
		case Parity_Mode_ODD:
			if(count%2){
				return ((data<<1)|1);
			} else {
				return (data<<1);
			}
			break;
		case Parity_Mode_EVENT:
			if(!(count%2)){
				return ((data<<1)|1);
			} else {
				return (data<<1);
			}
			break;
		default:
			return data;
			break;
	}
}


uint8_t Parity_Check(uint8_t data, Parity_Mode Mode){
	uint8_t count =0;
	for(int i=0; i< 8; i++){
		if(data & 0x01){
			count++;
		}
		data>>=1;
	}
	switch(Mode){
		case Parity_Mode_NONE:
			return 1; 
			break;
		case Parity_Mode_ODD:
			return (count%2); 
			break;
		case Parity_Mode_EVENT:
			return (!(count%2)); 
			break;
		default:
			return 0;
			break;
	}
}




void delay_us(uint8_t timedelay)
	{
			TIM_SetCounter(TIM2,0);
			while(TIM_GetCounter(TIM2)<timedelay*10){}
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


void GPIO_Config(){
GPIO_InitTypeDef GPIOInitStruct;
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIOInitStruct);
	//
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIOInitStruct);
}


void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}


void UART_Config(){
	GPIO_SetBits(UART_GPIO, TX_Pin);
	delay_us(1);
}


void UART_Transmit(uint8_t DataValue)
{
	GPIO_WriteBit(UART_GPIO, TX_Pin, Bit_RESET);
	delay_us(BRateTime);
	for ( unsigned char i = 0; i < 8; i++ ){
		if( ((DataValue)&0x1) == 0x1 ){
			GPIO_WriteBit(UART_GPIO, TX_Pin, Bit_SET);
		} else{
			GPIO_WriteBit(UART_GPIO, TX_Pin, Bit_RESET);
		}
		delay_us(BRateTime);
		DataValue = DataValue>>1;
	}
	// Send Stop Bit
	GPIO_WriteBit(UART_GPIO, TX_Pin, Bit_SET);
	delay_us(BRateTime);
}


unsigned char UART_Receive(void){
	unsigned char DataValue = 0;
	while(GPIO_ReadInputDataBit(UART_GPIO, RX_Pin) == 1);
	delay_us(BRateTime);
	delay_us(BRateTime/2);
	for ( unsigned char i = 0; i < 8; i++ ){
		if ( GPIO_ReadInputDataBit(UART_GPIO, RX_Pin) == 1 ){
			DataValue |= (1<<i);}
		delay_us(BRateTime);
	}
	if ( GPIO_ReadInputDataBit(UART_GPIO, RX_Pin) == 1 ){
		delay_us(BRateTime/2);
		return DataValue;
	}
	
}




uint8_t data = 0x02;
int main()
	{
		UART_Transmit(Parity_Generate(data, Parity_Mode_ODD));
		while(1)
		{
			
		}
		}