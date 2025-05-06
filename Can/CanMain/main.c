#include "stm32f10x.h"                  // Device header
#include "stm32f10x_can.h"              // Device:StdPeriph Drivers:CAN
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC

uint16_t ID = 0x023; 

void RCC_Config(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

void CAN_Config(){
	CAN_InitTypeDef CAN_InitStruct;
	
	CAN_InitStruct.CAN_TTCM = DISABLE;
	CAN_InitStruct.CAN_ABOM = ENABLE;
	CAN_InitStruct.CAN_AWUM = ENABLE;
	CAN_InitStruct.CAN_NART = DISABLE;
	CAN_InitStruct.CAN_RFLM = ENABLE;
	CAN_InitStruct.CAN_TXFP = ENABLE;
	CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
	
	//Cau hinh toc do can.
	//36Mhz
	//fCan = 36000000/4 = 9 000 000.
	//1tq = 1/9000000.
	//Toc do CAN = 1/(9tq) = 1Mb/s
	CAN_InitStruct.CAN_Prescaler = 4;
	CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStruct.CAN_BS1 = CAN_BS1_1tq;
	CAN_InitStruct.CAN_BS2 = CAN_BS2_1tq;
	
	CAN_Init(CAN1, &CAN_InitStruct);
}


void CAN_Filter_Config(){
	CAN_FilterInitTypeDef FilterInitStruct;
	
	FilterInitStruct.CAN_FilterNumber = 0;
	FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
	FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
	FilterInitStruct.CAN_FilterMaskIdHigh = 0b1111111111110000; // 0xFFE0
	FilterInitStruct.CAN_FilterMaskIdLow = 0x0000;
	FilterInitStruct.CAN_FilterIdHigh = ID << 5;
	FilterInitStruct.CAN_FilterIdLow = 0x0000;
	FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	FilterInitStruct.CAN_FilterActivation = ENABLE;
	
	CAN_FilterInit(&FilterInitStruct);
}

void CAN_SendData(uint8_t *data, uint8_t len)
{
	CanTxMsg TxMessage;

    TxMessage.StdId = ID; // 11bit ID voi che do std
    TxMessage.ExtId = 0x00;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.DLC = len;
	
    for (int i = 0; i < len; i++)
    {
        TxMessage.Data[i] = data[i];
    }

    CAN_Transmit(CAN1, &TxMessage);
	
}


uint8_t TestArray[20];
void CAN_RecvData(void){
		CanRxMsg RxMessage;
    while (CAN_MessagePending(CAN1, CAN_FIFO0) <1 );
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		ID = RxMessage.StdId;
			for (int i = 0; i < RxMessage.DLC; i++)
			{
					TestArray[i] = RxMessage.Data[i];

			}
}

int main()
{
		
}