#include <string.h>
#include "serial.h"
#include "UART.h"
#include "stm32f4xx.h"
#include <stdlib.h>
bool g_bDataAvailable  = false;
uint8_t g_data[6] = {0};
uint8_t g_cmd[1] = {0};
uint8_t STX[] = {0x02U};
uint8_t ETX[] = {0x03U};
uint8_t ACK[] = {0x06U};
uint8_t SYN[] = {0x16U};
void writeCom(uint8_t* cmd, uint8_t* data){
	uint8_t index = 0;
	memcpy(U2_TxBuffer + index, STX, 1);
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


	
