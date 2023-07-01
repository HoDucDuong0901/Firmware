#ifndef __UART_H_
#define __UART_H_
#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdint.h>
/*-------- Hardware config USART1 (can change) ---------
 * @brief:	Baudrate 115200
 *			Used to communicate between MCU & IMU
 * @pinout: PB6: TX -> RX IMU
 *					PB7: RX -> TX IMU
 */
#define		U1_Baudrate									115200
#define 	U1_GPIOx										GPIOB
#define 	U1_GPIO_Pin_Tx        			GPIO_Pin_6
#define 	U1_GPIO_Pin_Rx							GPIO_Pin_7
#define		U1_GPIO_PinSourceTx					GPIO_PinSource6
#define		U1_GPIO_PinSourceRx					GPIO_PinSource7
#define 	U1_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOB
/*-------- Hardware config USART2 (can change) ---------
 * @brief:	Baudrate 115200
 *			Used to communicate between LORA-GPS & LORA-BASE
 * @pinout:	PD5: TX -> RX ROVER-GPS 
 *					PD6: RX -> TX ROVER-GPS
 */
#define		U2_Baudrate									115200
#define 	U2_GPIOx						    		GPIOD
#define 	U2_GPIO_Pin_Tx        			GPIO_Pin_5
#define 	U2_GPIO_Pin_Rx							GPIO_Pin_6
#define		U2_GPIO_PinSourceTx					GPIO_PinSource5
#define		U2_GPIO_PinSourceRx					GPIO_PinSource6
#define 	U2_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOD
  /*-------- Hardware config USART6 ---------
 * @brief:	Baudrate 115200
 *			Used to communicate between LORA-GPS & LORA-BASE
 * @pinout:	PC6: TX -> Rx Lora MCU
 *					PC7: RX -> TX Lora MCU
 */
#define	  U6_Baudrate								  115200
#define 	U6_GPIOx										GPIOC
#define 	U6_GPIO_Pin_Tx        			GPIO_Pin_6
#define 	U6_GPIO_Pin_Rx							GPIO_Pin_7
#define		U6_GPIO_PinSourceTx					GPIO_PinSource6
#define		U6_GPIO_PinSourceRx					GPIO_PinSource7
#define 	U6_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOC
#define   IMU_TX_BUFFERSIZE 60
#define   IMU_RX_BUFFERSIZE 100
extern 		uint8_t 	U1_TxBuffer[IMU_TX_BUFFERSIZE], U1_RxBuffer[IMU_RX_BUFFERSIZE];
#define   ROVER_TX_BUFFERSIZE 0
#define   ROVER_RX_BUFFERSIZE 100

#define LORA_RX_BUFFERSIZE	10 
#define LORA_TX_BUFFERSIZE 	10
extern 		uint8_t 	U2_TxBuffer[LORA_TX_BUFFERSIZE], U2_RxBuffer[LORA_RX_BUFFERSIZE];
extern 		uint8_t 	U6_RxBuffer[ROVER_RX_BUFFERSIZE], U6_TxBuffer[LORA_TX_BUFFERSIZE]; // +1 for NULL-terminated

extern bool	g_bDataAvailable;
extern uint8_t g_data[6];
extern uint8_t g_cmd[1];
void writeCom(uint8_t* cmd, uint8_t* data);
/*export Function*/
void 	USART1_Config(uint32_t  BaudRate);
void 	USART2_Config(uint32_t  BaudRate);
void 	USART6_Config(uint32_t  BaudRate);
//void 	U1_SendData(uint16_t NbOfByte);
void 	U6_SendData(uint16_t NbOfByte);

#endif
