#include "UART.h"
/* Variables */
uint8_t 	U1_TxBuffer[IMU_TX_BUFFERSIZE]   =   {0}, U1_RxBuffer[IMU_RX_BUFFERSIZE]   = {0};
uint8_t		U2_TxBuffer[ROVER_TX_BUFFERSIZE] =   {0}, U2_RxBuffer[ROVER_RX_BUFFERSIZE] = {0};
uint8_t		U6_TxBuffer[LORA_TX_BUFFERSIZE]  =   {0}, U6_RxBuffer[LORA_RX_BUFFERSIZE]  = {0};
uint8_t		U4_TxBuffer[MCU_TX_BUFFERSIZE]   =	 {0}, U4_RxBuffer[MCU_RX_BUFFERSIZE] 	 = {0};
bool g_bDataAvailable  = false;
uint8_t		bCmd[4] = {0};
uint8_t 	STX[] = {0x02U};
uint8_t 	ETX[] = {0x03U};
uint8_t 	ACK[] = {0x06U};
uint8_t 	SYN[] = {0x16U};
GPIO_InitTypeDef							US_GPIO_Struct;
DMA_InitTypeDef								US_DMA_Struct;
USART_InitTypeDef							US_USART_Struct;
NVIC_InitTypeDef							US_NVIC_Struct;
void USART1_Config(uint32_t  BaudRate)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);		// USART clock
	RCC_AHB1PeriphClockCmd(U1_RCC_AHB1Periph_GPIOx, ENABLE);	// GPIO clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);		  // DMA clock

	US_GPIO_Struct.GPIO_Pin   = U1_GPIO_Pin_Tx|U1_GPIO_Pin_Rx;
	US_GPIO_Struct.GPIO_Mode  = GPIO_Mode_AF;
	US_GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	US_GPIO_Struct.GPIO_PuPd  = GPIO_PuPd_UP;
	US_GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(U1_GPIOx, &US_GPIO_Struct);
	
	GPIO_PinAFConfig(U1_GPIOx, U1_GPIO_PinSourceTx, GPIO_AF_USART1);
	GPIO_PinAFConfig(U1_GPIOx, U1_GPIO_PinSourceRx, GPIO_AF_USART1);
	
	// Config DMA USART Tx 
	US_USART_Struct.USART_BaudRate            = BaudRate;
	US_USART_Struct.USART_Mode                = USART_Mode_Tx|USART_Mode_Rx;
	US_USART_Struct.USART_WordLength          = USART_WordLength_8b;
	US_USART_Struct.USART_Parity              = USART_Parity_No;
	US_USART_Struct.USART_StopBits            = USART_StopBits_1;
	US_USART_Struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &US_USART_Struct);
	USART_Cmd(USART1, ENABLE);	// Enable USART1
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	// Enable USART1 Tx DMA
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	// Enable USART1 Rx DMA

	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
	US_NVIC_Struct.NVIC_IRQChannel = USART1_IRQn;
	US_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
	US_NVIC_Struct.NVIC_IRQChannelSubPriority = 1;
	US_NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&US_NVIC_Struct);

	//-----------Config DMA USART1 Tx: DMA2_Stream7, Channel 4------------------
	US_DMA_Struct.DMA_Channel            = DMA_Channel_4;
	US_DMA_Struct.DMA_BufferSize         = IMU_TX_BUFFERSIZE;
	US_DMA_Struct.DMA_Mode               = DMA_Mode_Normal;
	US_DMA_Struct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
	US_DMA_Struct.DMA_Memory0BaseAddr    = (uint32_t)&U1_TxBuffer;
	US_DMA_Struct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	US_DMA_Struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	US_DMA_Struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	US_DMA_Struct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	US_DMA_Struct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
	US_DMA_Struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	US_DMA_Struct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	US_DMA_Struct.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	US_DMA_Struct.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
	US_DMA_Struct.DMA_Priority           = DMA_Priority_Medium;
	DMA_Init(DMA2_Stream7, &US_DMA_Struct);
	//-----------Config DMA USART1 Rx: DMA2_Stream5, Channel 4------------------
	US_DMA_Struct.DMA_Channel            = DMA_Channel_4;
	US_DMA_Struct.DMA_BufferSize         = IMU_RX_BUFFERSIZE;
	US_DMA_Struct.DMA_Mode               = DMA_Mode_Circular;
	US_DMA_Struct.DMA_DIR                = DMA_DIR_PeripheralToMemory;
	US_DMA_Struct.DMA_Memory0BaseAddr    = (uint32_t)&U1_RxBuffer;	
	US_DMA_Struct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	US_DMA_Struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	US_DMA_Struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	US_DMA_Struct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	US_DMA_Struct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
	US_DMA_Struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	US_DMA_Struct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	US_DMA_Struct.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	US_DMA_Struct.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
	US_DMA_Struct.DMA_Priority           = DMA_Priority_Medium;
	DMA_Init(DMA2_Stream5, &US_DMA_Struct);
	DMA_Cmd(DMA2_Stream5, ENABLE);

	//------------NVIC Config----------
	US_NVIC_Struct.NVIC_IRQChannel                   = DMA2_Stream5_IRQn;
	US_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
	US_NVIC_Struct.NVIC_IRQChannelSubPriority        = 1;
	US_NVIC_Struct.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&US_NVIC_Struct);
	DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
}
void USART6_Config(uint32_t BaudRate)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	RCC_AHB1PeriphClockCmd(U6_RCC_AHB1Periph_GPIOx, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	US_GPIO_Struct.GPIO_Pin   = U6_GPIO_Pin_Tx|U6_GPIO_Pin_Rx;
	US_GPIO_Struct.GPIO_Mode  = GPIO_Mode_AF;
	US_GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	US_GPIO_Struct.GPIO_PuPd  = GPIO_PuPd_UP;
	US_GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(U6_GPIOx, &US_GPIO_Struct);
	
	GPIO_PinAFConfig(U6_GPIOx,U6_GPIO_PinSourceTx,GPIO_AF_USART6);
	GPIO_PinAFConfig(U6_GPIOx,U6_GPIO_PinSourceRx,GPIO_AF_USART6);
	
	//Config USART Tx 
	US_USART_Struct.USART_BaudRate            = BaudRate;
	US_USART_Struct.USART_Mode                = USART_Mode_Tx|USART_Mode_Rx;
	US_USART_Struct.USART_WordLength          = USART_WordLength_8b;
	US_USART_Struct.USART_Parity              = USART_Parity_No;
	US_USART_Struct.USART_StopBits 	          = USART_StopBits_1;
	US_USART_Struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6, &US_USART_Struct);
	USART_Cmd(USART6, ENABLE);
	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);

//	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
//	US_NVIC_Struct.NVIC_IRQChannel = USART6_IRQn;
//	US_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
//	US_NVIC_Struct.NVIC_IRQChannelSubPriority = 3;
//	US_NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&US_NVIC_Struct);

	//--------Config DMA USART6 Tx: DMA2_Stream6, Channel 5----------
	US_DMA_Struct.DMA_Channel            = DMA_Channel_5;
	US_DMA_Struct.DMA_BufferSize         = LORA_TX_BUFFERSIZE;
	US_DMA_Struct.DMA_Mode               = DMA_Mode_Normal;
	US_DMA_Struct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
	US_DMA_Struct.DMA_Memory0BaseAddr    = (uint32_t)&U6_TxBuffer;
	US_DMA_Struct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	US_DMA_Struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	US_DMA_Struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	US_DMA_Struct.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
	US_DMA_Struct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
	US_DMA_Struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	US_DMA_Struct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	US_DMA_Struct.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	US_DMA_Struct.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
	US_DMA_Struct.DMA_Priority           = DMA_Priority_VeryHigh;
	DMA_Init(DMA2_Stream6, &US_DMA_Struct);
	//-----------Config DMA USART Rx: DMA2_Stream2, Channel 5------------------
	US_DMA_Struct.DMA_Channel            = DMA_Channel_5;
	US_DMA_Struct.DMA_BufferSize         = LORA_RX_BUFFERSIZE;
	US_DMA_Struct.DMA_Mode               = DMA_Mode_Normal;
	US_DMA_Struct.DMA_DIR                = DMA_DIR_PeripheralToMemory;
	US_DMA_Struct.DMA_Memory0BaseAddr    = (uint32_t)&U6_RxBuffer;
	US_DMA_Struct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	US_DMA_Struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	US_DMA_Struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	US_DMA_Struct.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
	US_DMA_Struct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
	US_DMA_Struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	US_DMA_Struct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	US_DMA_Struct.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	US_DMA_Struct.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
	US_DMA_Struct.DMA_Priority           = DMA_Priority_VeryHigh;
	DMA_Init(DMA2_Stream2, &US_DMA_Struct);
	DMA_Cmd(DMA2_Stream2, ENABLE);

	//------------NVIC Config----------
	US_NVIC_Struct.NVIC_IRQChannel                   = DMA2_Stream2_IRQn;
	US_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
	US_NVIC_Struct.NVIC_IRQChannelSubPriority        = 3;
	US_NVIC_Struct.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&US_NVIC_Struct);
	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
}
void USART2_Config(uint32_t  BaudRate){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_AHB1PeriphClockCmd(U2_RCC_AHB1Periph_GPIOx,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	
	US_GPIO_Struct.GPIO_Pin   = U2_GPIO_Pin_Tx|U2_GPIO_Pin_Rx;
	US_GPIO_Struct.GPIO_Mode  = GPIO_Mode_AF;
	US_GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	US_GPIO_Struct.GPIO_PuPd  = GPIO_PuPd_UP;
	US_GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(U2_GPIOx,&US_GPIO_Struct);
	
	GPIO_PinAFConfig(U2_GPIOx, U2_GPIO_PinSourceTx, GPIO_AF_USART2);
	GPIO_PinAFConfig(U2_GPIOx, U2_GPIO_PinSourceRx, GPIO_AF_USART2);
	
		//Config USART Tx 
	US_USART_Struct.USART_BaudRate            = BaudRate;
	US_USART_Struct.USART_Mode 	              = USART_Mode_Tx|USART_Mode_Rx;
	US_USART_Struct.USART_WordLength          = USART_WordLength_8b;
	US_USART_Struct.USART_Parity              = USART_Parity_No;
	US_USART_Struct.USART_StopBits            = USART_StopBits_1;
	US_USART_Struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_Init(USART2, &US_USART_Struct);
	USART_Cmd(USART2, ENABLE);
	
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	US_NVIC_Struct.NVIC_IRQChannel = USART2_IRQn;
	US_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
	US_NVIC_Struct.NVIC_IRQChannelSubPriority = 2;
	US_NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&US_NVIC_Struct);
	//-----------Config DMA USART2 Tx: DMA1_Stream6, Channel 4------------------
//	US_DMA_Struct.DMA_Channel            = DMA_Channel_4;
//	US_DMA_Struct.DMA_BufferSize         = ROVER_TX_BUFFERSIZE;
//	US_DMA_Struct.DMA_Mode               = DMA_Mode_Normal;
//	US_DMA_Struct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
//	US_DMA_Struct.DMA_Memory0BaseAddr    = (uint32_t)&U2_TxBuffer;
//	US_DMA_Struct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
//	US_DMA_Struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
//	US_DMA_Struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
//	US_DMA_Struct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
//	US_DMA_Struct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
//	US_DMA_Struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	US_DMA_Struct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
//	US_DMA_Struct.DMA_FIFOMode           = DMA_FIFOMode_Disable;
//	US_DMA_Struct.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
//	US_DMA_Struct.DMA_Priority           = DMA_Priority_High;
//	DMA_Init(DMA1_Stream6, &US_DMA_Struct);
	//-----------Config DMA USART Rx: DMA1_Stream5, Channel 4------------------
	US_DMA_Struct.DMA_Channel            = DMA_Channel_4;
	US_DMA_Struct.DMA_BufferSize         = ROVER_RX_BUFFERSIZE ;
	US_DMA_Struct.DMA_Mode               = DMA_Mode_Circular;
	US_DMA_Struct.DMA_DIR                = DMA_DIR_PeripheralToMemory;
	US_DMA_Struct.DMA_Memory0BaseAddr    = (uint32_t)&U2_RxBuffer;
	US_DMA_Struct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	US_DMA_Struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	US_DMA_Struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	US_DMA_Struct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
	US_DMA_Struct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
	US_DMA_Struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	US_DMA_Struct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	US_DMA_Struct.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	US_DMA_Struct.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
	US_DMA_Struct.DMA_Priority           = DMA_Priority_High;
	DMA_Init(DMA1_Stream5, &US_DMA_Struct);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	//------------NVIC Config Rx----------
	US_NVIC_Struct.NVIC_IRQChannel                   = DMA1_Stream5_IRQn;
	US_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
	US_NVIC_Struct.NVIC_IRQChannelSubPriority        = 0;
	US_NVIC_Struct.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&US_NVIC_Struct);
	DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
	//------------NVIC Config Tx----------
//	US_NVIC_Struct.NVIC_IRQChannel                   = DMA1_Stream6_IRQn;
//	US_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
//	US_NVIC_Struct.NVIC_IRQChannelSubPriority        = 1;
//	US_NVIC_Struct.NVIC_IRQChannelCmd                = ENABLE;
//	NVIC_Init(&US_NVIC_Struct);
//	DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
}
void  UART4_Config(uint32_t 	BaudRate){
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	RCC_AHB1PeriphClockCmd(U4_RCC_AHB1Periph_GPIOx,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	
	US_GPIO_Struct.GPIO_Pin   = U4_GPIO_Pin_Tx|U4_GPIO_Pin_Rx;
	US_GPIO_Struct.GPIO_Mode  = GPIO_Mode_AF;
	US_GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	US_GPIO_Struct.GPIO_PuPd  = GPIO_PuPd_UP;
	US_GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(U4_GPIOx,&US_GPIO_Struct);
	
	GPIO_PinAFConfig(U4_GPIOx, U4_GPIO_PinSourceTx, GPIO_AF_UART4);
	GPIO_PinAFConfig(U4_GPIOx, U4_GPIO_PinSourceRx, GPIO_AF_UART4);
	
		//Config USART Tx 
	US_USART_Struct.USART_BaudRate            = BaudRate;
	US_USART_Struct.USART_Mode 	              = USART_Mode_Tx|USART_Mode_Rx;
	US_USART_Struct.USART_WordLength          = USART_WordLength_8b;
	US_USART_Struct.USART_Parity              = USART_Parity_No;
	US_USART_Struct.USART_StopBits            = USART_StopBits_1;
	US_USART_Struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_Init(UART4, &US_USART_Struct);
	USART_Cmd(UART4, ENABLE);
	
	USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
	US_NVIC_Struct.NVIC_IRQChannel = UART4_IRQn;
	US_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
	US_NVIC_Struct.NVIC_IRQChannelSubPriority = 4;
	US_NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&US_NVIC_Struct);
	//-----------Config DMA UART4 Tx: DMA1_Stream4, Channel 4------------------
	US_DMA_Struct.DMA_Channel            = DMA_Channel_4;
	US_DMA_Struct.DMA_BufferSize         = MCU_TX_BUFFERSIZE;
	US_DMA_Struct.DMA_Mode               = DMA_Mode_Normal;
	US_DMA_Struct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
	US_DMA_Struct.DMA_Memory0BaseAddr    = (uint32_t)&U4_TxBuffer;
	US_DMA_Struct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	US_DMA_Struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	US_DMA_Struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	US_DMA_Struct.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
	US_DMA_Struct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
	US_DMA_Struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	US_DMA_Struct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	US_DMA_Struct.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	US_DMA_Struct.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
	US_DMA_Struct.DMA_Priority           = DMA_Priority_Medium;
	DMA_Init(DMA1_Stream4, &US_DMA_Struct);
	//DMA_Cmd(DMA1_Stream4, ENABLE);
	//-----------Config DMA UART4 Rx: DMA1_Stream2, Channel 4------------------
	US_DMA_Struct.DMA_Channel            = DMA_Channel_4;
	US_DMA_Struct.DMA_BufferSize         = MCU_RX_BUFFERSIZE ;
	US_DMA_Struct.DMA_Mode               = DMA_Mode_Circular;
	US_DMA_Struct.DMA_DIR                = DMA_DIR_PeripheralToMemory;
	US_DMA_Struct.DMA_Memory0BaseAddr    = (uint32_t)&U4_RxBuffer;
	US_DMA_Struct.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	US_DMA_Struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	US_DMA_Struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	US_DMA_Struct.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
	US_DMA_Struct.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
	US_DMA_Struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	US_DMA_Struct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	US_DMA_Struct.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	US_DMA_Struct.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
	US_DMA_Struct.DMA_Priority           = DMA_Priority_Medium;
	DMA_Init(DMA1_Stream2, &US_DMA_Struct);
	DMA_Cmd(DMA1_Stream2, ENABLE);
	//------------NVIC Config Rx----------
	US_NVIC_Struct.NVIC_IRQChannel                   = DMA1_Stream2_IRQn;
	US_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
	US_NVIC_Struct.NVIC_IRQChannelSubPriority        = 4;
	US_NVIC_Struct.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&US_NVIC_Struct);
	DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
	//------------NVIC Config Tx----------
	US_NVIC_Struct.NVIC_IRQChannel                   = DMA1_Stream4_IRQn;
	US_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 0;
	US_NVIC_Struct.NVIC_IRQChannelSubPriority        = 4;
	US_NVIC_Struct.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&US_NVIC_Struct);
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
}


void writeCom(uint8_t* cmd, uint8_t* data){
	uint8_t index = 0;
	memcpy(U2_TxBuffer + index,STX, 1);
	index += 1;
	memcpy(U2_TxBuffer + index,cmd,1);
	index += 1;
	memcpy(U2_TxBuffer + index,data,6);
	index += 6;
	memcpy(U2_TxBuffer + index,ACK,1);
	index += 1;
	memcpy(U2_TxBuffer + index,ETX,1);
	DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
	DMA1_Stream6->NDTR = 10;
	DMA_Cmd(DMA1_Stream6, ENABLE);
}
void  U1_SendData(uint16_t NbOfByte){
	DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
	DMA2_Stream7->NDTR = NbOfByte;
	DMA_Cmd(DMA2_Stream7, ENABLE);
}
void U4_SendData(uint16_t NbOfByte){
	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
	DMA1_Stream4->NDTR = NbOfByte;
	DMA_Cmd(DMA1_Stream4, ENABLE);
}
void U6_SendData(uint16_t NbOfByte){
	DMA_ClearITPendingBit(DMA2_Stream6,DMA_IT_TCIF6);
	DMA2_Stream6->NDTR = 84;
	DMA_Cmd(DMA2_Stream6,ENABLE);
	
}
