#ifndef __CONFIG_H_
#define __CONFIG_H_
#include "stm32f4xx.h"
#include <stdbool.h>
void GPIO_Config(void);
void InTimer_Config(void);
void Encoder_Config(void);
void PWM_Config(void);
void UART_Config(void);
void DMA_Config(void);
#endif
