#ifndef __ENCODER_H_
#define __ENCODER_H_
#include "stm32f4xx.h"
/*----- Parameter define ----------------*/
#define         PWM_FREQUENCY       20000 // 20khz
/*----- Hardware Encoder config M1: PA6, PA7 ------*/
#define 				M1_TIMx												TIM3
#define					M1_GPIOx											GPIOA
#define					M1_RCC_PeriphClock						RCC_APB1Periph_TIM3
#define					M1_RCC_AHB1Periph_GPIOx				RCC_AHB1Periph_GPIOA
#define					M1_GPIO_Pin_x1								GPIO_Pin_6	// PA6 = TIM3_CH1 M1
#define					M1_GPIO_Pin_x2								GPIO_Pin_7	// PA7 = TIM3_CH2 M1
#define					M1_GPIO_AF_TIMx								GPIO_AF_TIM3
#define					M1_GPIO_PinSourcex1						GPIO_PinSource6
#define					M1_GPIO_PinSourcex2						GPIO_PinSource7
#define					M1_TIMx_IRQn									TIM3_IRQn
/*----- Hardware Encoder config M2: PD12, PD13 ------*/
#define 				M2_TIMx												TIM4
#define					M2_GPIOx											GPIOD
#define					M2_RCC_PeriphClock						RCC_APB1Periph_TIM4
#define					M2_RCC_AHB1Periph_GPIOx				RCC_AHB1Periph_GPIOD
#define					M2_GPIO_Pin_x1								GPIO_Pin_12 // PD12 = TIM3_CH1 M2
#define					M2_GPIO_Pin_x2								GPIO_Pin_13 // PD13 = TIM3_CH2 M2
#define					M2_GPIO_AF_TIMx								GPIO_AF_TIM4	
#define					M2_GPIO_PinSourcex1						GPIO_PinSource12
#define					M2_GPIO_PinSourcex2						GPIO_PinSource13
#define					M2_TIMx_IRQn									TIM4_IRQn
/*----- PWM configure ----- TIM9 */ 
#define					PWM_TIMx											TIM9
#define					PWM_GPIOx										  GPIOA
#define					PWM_GPIO_Pin_OC1							GPIO_Pin_2 // TIM9_CH1 for generating PWM pulse M1
#define					PWM_GPIO_Pin_OC2							GPIO_Pin_3 // TIM9_CH2 for generating PWM pulse M2
#define					PWM_GPIO_AF_TIMx							GPIO_AF_TIM9
#define					PWM_GPIO_PinSourceOC1					GPIO_PinSource2
#define					PWM_GPIO_PinSourceOC2					GPIO_PinSource3
#define					PWM_RCC_PeriphClock						RCC_APB2Periph_TIM9
#define					PWM_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOA
/*----- Dir pin configure ----- PC3 PC4 */
#define					Dir_GPIOx											GPIOC
#define					Dir_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOC
#define					Dir_GPIO_Pin_M1								GPIO_Pin_3 //Dir pin for M1: PC3=DIR+ M1; GND=DIR- M1
#define					Dir_GPIO_Pin_M2								GPIO_Pin_4 //Dir pin for M2: PC4=DIR+ M1; GND=DIR- M2

void Encoder_Config_M1(void);
void Encoder_Config_M2(void);
void PWM_Config(void);
void InTimer_Config(void);
void GPIO_Config(void);
#endif
