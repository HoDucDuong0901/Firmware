#include "stm32f4xx.h"                  // Device header
#include "system_timetick.h"
#include <string.h>
#include "UART.h"
#include "encoder.h"
#include "motor.h"
/* --------- ----------------Peripherals--------------------
---- TIM3: Encoder Motor1 PA6(TIM3_CH1)Grey, PA7(TIM3_CH2)Pink // RIGHT SIDE ( Behind view )
---- TIM4: Encoder Motor2 PD12(TIM4_CH1)Blue, PD13(TIM4_CH2) Red // LEFT SIDE ( Behind view )
---- GPIO: PC3 for motor1, PC4 for motor2
---- USART6: PC6(Tx), PC7(Rx) ( for communicating between LORA-MCU & LORA-PC )
---- USART2: PD5(Tx), PD6(Rx) ( for communicating between GPS Rover & GPS Base )
---- USART1: PB6(Tx), PB7(Rx) ( for communicating between MCU & IMU )
---- TIM9_CH1: PA2=PWM+ M1; GND=PWM- M1
---- TIM9_CH2: PA3=PWM+ M2; GND=PWM- M2
------------------------------------------------------------*/
DC_MOTOR MOTOR1 = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
DC_MOTOR MOTOR2 = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

uint8_t data[10] = {0};

	int main(){
		SysTick_Config(SystemCoreClock / 100); // value for 24-bit down counter register
		USART2_Config(U2_Baudrate);
		Encoder_Config_M1();
		Encoder_Config_M2();
		GPIO_Config();
		PWM_Config();
		PID_ResetPID(&MOTOR1);
		PID_ResetPID(&MOTOR2);
		InTimer_Config();	
		while(1){
			if(g_bDataAvailable == true){
				if(U2_RxBuffer[1] == 'S'){
						PID_ResetPID(&MOTOR2);
						Counter_Reset(&MOTOR2);
						TIM_SetCounter(M2_TIMx, 0);
						TIM_SetCompare2(PWM_TIMx,0);
						MOTOR2.dKp = (double)((U2_RxBuffer[2] << 8) + U2_RxBuffer[3])*0.01; 
						MOTOR2.dKi = (double)((U2_RxBuffer[4] << 8) + U2_RxBuffer[5])*0.01; 
						MOTOR2.dKd = (double)((U2_RxBuffer[6] << 8) + U2_RxBuffer[7])*0.0001; 
						g_cmd[0] = 'S';
						memset(g_data,0,6);
						writeCom(g_cmd,g_data);
				}
				else if(U2_RxBuffer[1] == 'M'){
						Counter_Reset(&MOTOR2);
						writeCom(g_cmd,g_data);
						MOTOR2.dCurrent_set_v = (double)((U2_RxBuffer[2] << 8) + U2_RxBuffer[3]);
						g_cmd[0] = 'M';
					
				}
				else if(U2_RxBuffer[1] == 'E'){
						PID_ResetPID(&MOTOR2);
						Counter_Reset(&MOTOR2);
						TIM_SetCounter(M2_TIMx, 0);
						TIM_SetCompare2(PWM_TIMx,0);
						g_cmd[0] = 'E';
						memset(g_data,0,6);
						writeCom(g_cmd,g_data);
				}
					g_bDataAvailable = false;
			}
			if(robot_status.bveh_run == true){
					EncoderRead(&MOTOR1,M1_TIMx,&sTime);
					EncoderRead(&MOTOR2,M2_TIMx,&sTime);
					uint16_t uTemp = (uint16_t)MOTOR2.dCurrent_v;
					g_data[0] = (uTemp >> 8) & 0xFF;
					g_data[1] =  uTemp & 0xFF;
					//writeCom(g_cmd,g_data);
					PID_Compute(&MOTOR2,&sTime);
//				PID_Compute(&MOTOR2,&sTime);
					RobotRun(0,MOTOR2.dOut);
//				RobotRun(MOTOR2.dOut,0);
				if(g_cmd[0] == 'M')
					{
						if(tick_count > 50){
							{
								writeCom(g_cmd,g_data);
								tick_count = 0;
							}
					}
						robot_status.bveh_run = false;
			}
	}
}
}
// for Lora PC,MCU
void DMA1_Stream5_IRQHandler(void){
	DMA_ClearITPendingBit(DMA1_Stream5,DMA_IT_TCIF5);
	DMA1_Stream5->NDTR = LORA_RX_BUFFERSIZE;
	DMA_Cmd(DMA1_Stream5, ENABLE);
	if(U2_RxBuffer[0] == 0x02 &&  U2_RxBuffer[9] == 0x03){
		g_bDataAvailable = true;
	}
}
void DMA1_Stream6_IRQHandler(void){
	DMA_ClearITPendingBit(DMA1_Stream6,DMA_IT_TCIF6);
	DMA_Cmd(DMA1_Stream6, DISABLE);
}
void USART2_IRQHandler(void){
	if(USART_GetITStatus(USART2, USART_IT_IDLE)){
		USART_ClearFlag(USART2, USART_FLAG_IDLE);
		uint16_t temp = USART_ReceiveData(USART2);
		DMA_Cmd(DMA1_Stream5, DISABLE);
	}
}
void TIM2_IRQHandler(void){
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	//EncoderRead(&MOTOR1,TIM3,&sTime);
	//EncoderRead(&MOTOR2,TIM4,&sTime);
	//RobotRun(100,100);
	//MOTOR2.uEnc = TIM_GetCounter(TIM4);
}
