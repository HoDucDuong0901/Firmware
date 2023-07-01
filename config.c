#include "config.h"
#include "serial.h"
void GPIO_Config(void){
	// clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	// configure GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	///
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//
}
void Encoder_Config(void){
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 ,  ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9| GPIO_Pin_11;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1|TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising; 
	TIM_ICInitStructure.TIM_ICFilter=0;
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInit(TIM1, &TIM_ICInitStructure); 
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_Cmd( TIM1, ENABLE);
	TIM_SetCounter(TIM1, 0);
}
void InTimer_Config(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	
	NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	TIM_TimeBaseInitTypeDef TIM2_InitStructure;
	TIM2_InitStructure.TIM_ClockDivision = 0;
	TIM2_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM2_InitStructure.TIM_Period = 10000-1;
	TIM2_InitStructure.TIM_Prescaler = 84-1;
	TIM_TimeBaseInit(TIM2,&TIM2_InitStructure);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_Cmd( TIM2, ENABLE);
}

void PWM_Config(void){
		GPIO_InitTypeDef 					GPIO_InitStructure;
	  TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
		TIM_OCInitTypeDef 			  TIM_OCInitStructure;
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
		

		TIM_TimeBaseStructure.TIM_Period = 4200-1;
		TIM_TimeBaseStructure.TIM_Prescaler = 0;
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
		
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 0; // adjust the PWM duty.
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
		TIM_OC4Init(TIM3, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
		TIM_CtrlPWMOutputs(TIM3,ENABLE);
		TIM_Cmd(TIM3, ENABLE);
	}
void UART_Config(void){
		//GPIO Config
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	// Enable port
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4);
	//GPIO Config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// UART Config
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	USART_Cmd(UART4,ENABLE);
}
void DMA_Config(void){
	 DMA_InitTypeDef  DMA_InitStructure;
	 USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE); 
	 USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	 DMA_InitStructure.DMA_BufferSize = 18;
	 DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
	 DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	 DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
	 DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	 DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	 DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	 DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(UART4->DR)) ;
	 DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	 DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	 /* Configure TX DMA */
	 DMA_InitStructure.DMA_Channel =  DMA_Channel_4;
	 DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
   DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)nTxBuff;
   DMA_Init(DMA1_Stream4,&DMA_InitStructure);
	 //DMA_Cmd(DMA1_Stream4, ENABLE);
	
	  /* Configure RX DMA */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)g_nRxBuff ; 
  DMA_Init(DMA1_Stream2,&DMA_InitStructure);
	
	//IT_DMA Config
	 NVIC_InitTypeDef NVIC_InitStructure;
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	 NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);
	 DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
	 DMA_Cmd(DMA1_Stream2, ENABLE);
}
