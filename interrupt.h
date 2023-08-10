
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "motor.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/*  These function configure scheduler timer
    Sampling time is set by changing value PERIOD
    in the file "system_timetick.c"
*/

extern uint32_t tick_count;
extern uint32_t tick_flag;

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

// For USART1's interrution
void USART1_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void); // USART1 RX - IMU to MCU
void DMA2_Stream7_IRQHandler(void); // USART1 TX - IMU to MCU

// For USART2's interruption
void USART2_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void); // USART2 RX - GPS to MCU
void DMA1_Stream6_IRQHandler(void); // USART2 TX - GPS to MCU

// For USART6's interruption
void USART6_IRQHandler(void); 
void DMA2_Stream2_IRQHandler(void); // USART6 RX - Lora PC to Lora MCU
void DMA2_Stream6_IRQHandler(void); // USART6 TX - Lora PC to Lora MCU

// For TIM2's interruption
void TIM2_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */


