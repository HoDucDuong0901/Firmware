
/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "UART.h"
#include "functions.h"
#include "gps.h"
#include "motor.h"
uint32_t	tick_count;
uint32_t	tick_flag;
void NMI_Handler(void){}

void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1){}
}

void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}
	  
void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  tick_count++;
	robot_status.Vel_Status = Check_OK;
}

// For USART1's interrution
void USART1_IRQHandler(void){
	if(USART_GetITStatus(USART1, USART_IT_IDLE))
	{
		USART_ClearFlag(USART1, USART_FLAG_IDLE);
		uint16_t temp = USART_ReceiveData(USART1);
		DMA_Cmd(DMA2_Stream5, DISABLE);
	}
}
void DMA2_Stream5_IRQHandler(void) // USART1 RX - IMU to MCU
{
	DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
	if(U1_RxBuffer[0] == 0x0A && U1_RxBuffer[5] == 0x0D){
		uint16_t tmp = 0;
		tmp = U1_RxBuffer[3]*255 + U1_RxBuffer[4];
		Fuzzy.dAngle = U1_RxBuffer[2] + (double)tmp/10000;
		if(U1_RxBuffer[1] == 0x2D){
			Fuzzy.dAngle *= -1;
		}
	}
	DMA_Cmd(DMA2_Stream5, ENABLE);
}
void DMA2_Stream7_IRQHandler(void) // USART1 TX - MCU to IMU
{
	DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
}
	
	
// For USART2's interruption
void USART2_IRQHandler(void){
if(USART_GetITStatus(USART2, USART_IT_IDLE))
	{
		USART_ClearFlag(USART2, USART_FLAG_IDLE);
		uint16_t temp = USART_ReceiveData(USART2);
		DMA_Cmd(DMA1_Stream5, DISABLE);
	}
}
void DMA1_Stream5_IRQHandler(void) // USART2 RX - GPS to MCU
{
	DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
	GPS_M8P.GPS_status = GPS_NMEA_Message(&GPS_M8P,U2_RxBuffer,Message);
	if ( GPS_M8P.GPS_status == GPS_DataValid){
			robot_status.GPS_Status = Check_OK;
	}
	else{
		robot_status.GPS_Status = Check_NOK;
	}
	DMA_Cmd(DMA1_Stream5, ENABLE);
}	
void DMA1_Stream6_IRQHandler(void) // USART2 TX - GPS to MCU
{
	DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
}
	
// For UART4 interruption
void UART4_IRQHandler(void){
if(USART_GetITStatus(UART4, USART_IT_IDLE))
	{
		USART_ClearFlag(UART4, USART_FLAG_IDLE);
		uint16_t temp = USART_ReceiveData(UART4);
		DMA_Cmd(DMA1_Stream2, DISABLE);
	}	
	
}
void DMA1_Stream4_IRQHandler(void){ // UART4 Tx - MCU to Embedded PC
	DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
}

void DMA1_Stream2_IRQHandler(void){ // UART4 Rx- Embedded PC to MCU 
	DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
	
}


/*------------------- for USART6 interrupt ---------------------------*/
void DMA2_Stream2_IRQHandler(void) // USART6 RX - Lora PC to Lora MCU
{
	uint16_t uTemp = 0;
	DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);
	memcpy(bCmd,&U6_RxBuffer[1],4);
	if(U6_RxBuffer[0] == 0x02 && U6_RxBuffer[83] == 0x03){
		
		if (StringHeaderCompare((char*)bCmd,"SMAP",4)){
				ParsePoint(U6_RxBuffer,&GPS_M8P);
				ParseYaw(U6_RxBuffer,&GPS_M8P);
		}
		else if(StringHeaderCompare((char*)bCmd,"SPAR",4)){
			// Set PID parametes for motor 1
			uTemp = (U6_RxBuffer[5] << 8)  +  U6_RxBuffer[6];
			MOTOR1.dKp = (double)uTemp/100;
			uTemp = (U6_RxBuffer[7] << 8)  + U6_RxBuffer[8];
			MOTOR1.dKi = (double)uTemp/100;
			uTemp = (U6_RxBuffer[9] << 8)  + U6_RxBuffer[10];
			MOTOR1.dKd = (double)uTemp/10000;
			
			// Set PID parameters for motor 2
			uTemp = (U6_RxBuffer[11] << 8)  +  U6_RxBuffer[12];
			MOTOR2.dKp = (double)uTemp/100;
			uTemp = (U6_RxBuffer[13] << 8)  + U6_RxBuffer[14];
			MOTOR2.dKi = (double)uTemp/100;
			uTemp = (U6_RxBuffer[15] << 8)  + U6_RxBuffer[16];
			MOTOR2.dKd = (double)uTemp/10000;
			
			// Set Stanley parameters
			uTemp = (U6_RxBuffer[17] << 8) + U6_RxBuffer[18];
			GPS_M8P.dKgain = (double)uTemp/100;
			uTemp = (U6_RxBuffer[19] << 8) + U6_RxBuffer[20];
			GPS_M8P.dKsoft = (double)uTemp/100;
			
			// Set parameters for fuzzy 
			uTemp = (U6_RxBuffer[21] << 8) + U6_RxBuffer[22];
			Fuzzy.dKe = (double)uTemp/100;
			uTemp = (U6_RxBuffer[23] << 8) + U6_RxBuffer[24];
			Fuzzy.dKedot = (double)uTemp/100;
			uTemp = (U6_RxBuffer[25] << 8) + U6_RxBuffer[26];
			Veh.Max_Velocity = (double)uTemp/100;
		}
		else if(StringHeaderCompare((char*)bCmd,"VMOV",4)){
			robot_status.Veh_Auto_Flag = Check_OK;
		}
		else if(StringHeaderCompare((char*)bCmd,"STOP",4)){
			robot_status.Veh_Auto_Flag = Check_NOK;
		}
		else if(StringHeaderCompare((char*)bCmd,"CDAT",4)){
			
		}
	}
  DMA_Cmd(DMA2_Stream2, ENABLE);
}
void DMA2_Stream6_IRQHandler(void) // USART6 TX - MCU  to PC
{
	DMA_ClearITPendingBit(DMA2_Stream6,DMA_IT_TCIF6);
}
// For TIM2's interruption
void TIM2_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//	GPS_StanleyControl(&GPS_M8P,30.1,22.4);
//	send_Data();
	robot_status.Veh_SampleState = Check_OK;
}
	

