#include "stm32f10x.h"
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_spi.h"              // Device:StdPeriph Drivers:SPI
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM



#define SPI1_NSS 	GPIO_Pin_4
#define SPI1_SCK	GPIO_Pin_5
#define SPI1_MISO 	GPIO_Pin_6
#define SPI1_MOSI 	GPIO_Pin_7
#define SPI1_GPIO 	GPIOA

uint8_t buffer[7] = {1,3,4,7,2,4,5};

void RCC_Config(){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	}



void TIM_Config(){
	
		TIM_TimeBaseInitTypeDef TIM_TimBaseInitStructure;
		TIM_TimBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimBaseInitStructure.TIM_Period = 0xffff;
		TIM_TimBaseInitStructure.TIM_Prescaler = 72 - 1;
	
		TIM_TimeBaseInit(TIM2, &TIM_TimBaseInitStructure);
	
		TIM_Cmd(TIM2, ENABLE);
}

void delay_1ms(void)
{
	TIM_SetCounter(TIM2, 0);
	while(TIM_GetCounter(TIM2) < 100){}
	
}

void delay_ms(uint32_t time)
{
		while(time){
			
			delay_1ms();
			--time;
			}
}


void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = SPI1_NSS| SPI1_SCK| SPI1_MISO| SPI1_MOSI;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);
}


void SPI_Config(){
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;//72Mhs/16
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);
}




int main(){
		RCC_Config();
		GPIO_Config();
		SPI_Config();
		TIM_Config();
	
		while(1){
					for(int i =0; i<7; i++){
							SPI_I2S_SendData(SPI1, buffer[i]);
							while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);}
							delay_ms(300);
						}
			}

