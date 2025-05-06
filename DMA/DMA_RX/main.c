#include "stm32f10x.h"                  // Device header
#include "stm32f10x_dma.h"              // Device:StdPeriph Drivers:DMA
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_spi.h"              // Device:StdPeriph Drivers:SPI
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM


uint8_t data[10];

void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_SPI1| RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void DMA_Config()
{
		//DMA1, Channel 2, SPI1RX
		DMA_InitTypeDef DMA_InitStruct;

    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;   // SPI1 data register address
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)data;            // Memory address for the buffer
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;               // Direction: from peripheral to memory
    DMA_InitStruct.DMA_BufferSize = 10;                  // Number of data items to transfer
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // Do not increment peripheral address
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;          // Increment memory address after each transfer
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // Peripheral data size: 8 bits
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  // Memory data size: 8 bits
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;                  // Circular mode for continuous transfer
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;              // High priority
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;                     // Memory-to-memory mode disabled
	
	
    DMA_Init(DMA1_Channel2, &DMA_InitStruct);                     // Initialize DMA1 Channel2

    DMA_Cmd(DMA1_Channel2, ENABLE); // Enable DMA Channel2

    // Enable DMA requests for SPI1 RX
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

	
	
	
}


void GPIO_Config(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    // Configure SPI1 pins (PA5: SCK, PA6: MISO, PA7: MOSI)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function mode
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure NSS pin (PA4) for hardware NSS
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU; // Input mode with pull-up
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void SPI_Config(void) {
    SPI_InitTypeDef SPI_InitStruct;

    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // Full-duplex mode
    SPI_InitStruct.SPI_Mode = SPI_Mode_Slave;                       // Set SPI to Slave mode
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;                  // Data size: 8 bits
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;                         // Clock polarity low
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;                       // Clock phase: first edge
    SPI_InitStruct.SPI_NSS = SPI_NSS_Hard;                          // Hardware-controlled NSS
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;                 // Transmit most significant bit first
    SPI_Init(SPI1, &SPI_InitStruct);

    SPI_Cmd(SPI1, ENABLE); // Enable SPI1
}



int main()
{
		RCC_Config();
		SPI_Config();
		GPIO_Config();
		DMA_Config();
				while(1){
					
					}
}