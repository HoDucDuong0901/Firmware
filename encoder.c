#include "encoder.h"
TIM_TimeBaseInitTypeDef 			En_TIM_BaseStruct;
GPIO_InitTypeDef 						  En_GPIO_Struct;
NVIC_InitTypeDef						  En_NVIC_Struct;
void Encoder_Config_M1(void){
	RCC_AHB1PeriphClockCmd(M1_RCC_AHB1Periph_GPIOx, ENABLE);
	RCC_APB1PeriphClockCmd(M1_RCC_PeriphClock, ENABLE);
	/* Config PA6 PA7 as AF pin */
	En_GPIO_Struct.GPIO_Pin = M1_GPIO_Pin_x1 | M1_GPIO_Pin_x2;
	En_GPIO_Struct.GPIO_Mode = GPIO_Mode_AF;
	En_GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	En_GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	En_GPIO_Struct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(M1_GPIOx, &En_GPIO_Struct);
	GPIO_PinAFConfig(M1_GPIOx, M1_GPIO_PinSourcex1, M1_GPIO_AF_TIMx);
	GPIO_PinAFConfig(M1_GPIOx, M1_GPIO_PinSourcex2, M1_GPIO_AF_TIMx);
	/* Config Time Base */
	En_TIM_BaseStruct.TIM_Prescaler = 0;
	En_TIM_BaseStruct.TIM_Period = 0xFFFF;
	En_TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	En_TIM_BaseStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(M1_TIMx, &En_TIM_BaseStruct);
	TIM_ARRPreloadConfig(M1_TIMx, ENABLE);
	TIM_EncoderInterfaceConfig(M1_TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_TimeBaseInit(M1_TIMx, &En_TIM_BaseStruct);
	TIM_Cmd(M1_TIMx, ENABLE);
	TIM_SetCounter(M1_TIMx, 0);
}
void 	Encoder_Config_M2(void){
	RCC_AHB1PeriphClockCmd(M2_RCC_AHB1Periph_GPIOx, ENABLE);
	RCC_APB1PeriphClockCmd(M2_RCC_PeriphClock, ENABLE);
	/* Config PA8 as AF pin */
	En_GPIO_Struct.GPIO_Pin = M2_GPIO_Pin_x1|M2_GPIO_Pin_x2;
	En_GPIO_Struct.GPIO_Mode = GPIO_Mode_AF;
	En_GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	En_GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	En_GPIO_Struct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(M2_GPIOx,&En_GPIO_Struct);
	GPIO_PinAFConfig(M2_GPIOx, M2_GPIO_PinSourcex1, M2_GPIO_AF_TIMx);
	GPIO_PinAFConfig(M2_GPIOx, M2_GPIO_PinSourcex2, M2_GPIO_AF_TIMx);
	/* Config Time Base */
	En_TIM_BaseStruct.TIM_Prescaler = 0;
	En_TIM_BaseStruct.TIM_Period = 0xFFFF;
	En_TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	En_TIM_BaseStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(M2_TIMx, &En_TIM_BaseStruct); 
	TIM_ARRPreloadConfig(M2_TIMx, ENABLE); 
	TIM_EncoderInterfaceConfig(M2_TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_TimeBaseInit(M2_TIMx, &En_TIM_BaseStruct);
	TIM_Cmd(M2_TIMx, ENABLE);
	TIM_SetCounter(M2_TIMx, 0);	
}
void PWM_Config(void){
	TIM_OCInitTypeDef En_TIM_OCStruct;
	/* Enable RCC clock for each Peripheral */
	RCC_APB2PeriphClockCmd(PWM_RCC_PeriphClock, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_RCC_AHB1Periph_GPIOx, ENABLE);
	/* GPIO Config */
	En_GPIO_Struct.GPIO_Pin 		= 	PWM_GPIO_Pin_OC1|PWM_GPIO_Pin_OC2;
	En_GPIO_Struct.GPIO_Mode 		= 	GPIO_Mode_AF;
	En_GPIO_Struct.GPIO_OType 	= 	GPIO_OType_PP;
	En_GPIO_Struct.GPIO_Speed 	= 	GPIO_Speed_100MHz;
	En_GPIO_Struct.GPIO_PuPd 		= 	GPIO_PuPd_NOPULL;
	GPIO_Init(PWM_GPIOx,&En_GPIO_Struct);
		/* GPIO Config AF */
	GPIO_PinAFConfig(PWM_GPIOx, GPIO_PinSource2, PWM_GPIO_AF_TIMx);
	GPIO_PinAFConfig(PWM_GPIOx, GPIO_PinSource3, PWM_GPIO_AF_TIMx);

	/* Time Base Init */
	En_TIM_BaseStruct.TIM_Prescaler     = 0;
	En_TIM_BaseStruct.TIM_Period        = 8400 - 1;
	En_TIM_BaseStruct.TIM_CounterMode   = TIM_CounterMode_Up;
	En_TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(PWM_TIMx, &En_TIM_BaseStruct);
	/* PWM Config */
	En_TIM_OCStruct.TIM_OCMode      = TIM_OCMode_PWM1;
	En_TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	En_TIM_OCStruct.TIM_OCPolarity 	= TIM_OCPolarity_High;
	En_TIM_OCStruct.TIM_Pulse       = 0; // duty
	//
	TIM_OC1Init(PWM_TIMx, &En_TIM_OCStruct);
	TIM_OC1PreloadConfig(PWM_TIMx, TIM_OCPreload_Enable);

	//
	
	TIM_OC2Init(PWM_TIMx, &En_TIM_OCStruct);
	TIM_OC2PreloadConfig(PWM_TIMx, TIM_OCPreload_Enable);
	TIM_Cmd(PWM_TIMx,ENABLE);
}

void InTimer_Config(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	
	NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	TIM_TimeBaseInitTypeDef TIM2_InitStructure;
	TIM2_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM2_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM2_InitStructure.TIM_Period = 840000;
	TIM2_InitStructure.TIM_Prescaler = 0;
	TIM2_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&TIM2_InitStructure);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_SetCounter(TIM2,0);
	TIM_Cmd( TIM2, ENABLE);
}
void GPIO_Config(void){
	// clock enable
	RCC_AHB1PeriphClockCmd(Dir_RCC_AHB1Periph_GPIOx, ENABLE);
	// configure GPIO
	En_GPIO_Struct.GPIO_Pin = Dir_GPIO_Pin_M1|Dir_GPIO_Pin_M2; // PC3 for M1, PC4 for M2
	En_GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	En_GPIO_Struct.GPIO_Mode = GPIO_Mode_OUT;
	En_GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	En_GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(Dir_GPIOx, &En_GPIO_Struct);
}
