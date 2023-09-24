#include "stm32f4xx.h"                  // Device header
#include <string.h>
#include "UART.h"
#include "encoder.h"
#include "motor.h"
#include "functions.h"
/* --------- ----------------Peripherals--------------------
---- TIM3: Encoder Motor1 PA6(TIM3_CH1)Grey, PA7(TIM3_CH2)Pink // RIGHT SIDE ( Behind view )
---- TIM4: Encoder Motor2 PD12(TIM4_CH1)Blue, PD13(TIM4_CH2) Red // LEFT SIDE ( Behind view )
---- GPIO: PC3 for motor1, PC4 for motor2
---- USART6: PC6(Tx), PC7(Rx) ( for communication between LORA-MCU & LORA-PC ) DMA2
---- USART2: PD5(Tx), PD6(Rx) ( for communication between GPS Rover & GPS Base ) DMA1
---- USART1: PB6(Tx), PB7(Rx) ( for communication between MCU & IMU ) DMA2
---- USART4: PA0(Tx), PA1(Rx) ( for communication between MCU and Embedded PC) DMA1
---- TIM9_CH1: PA2=PWM+ M1; GND=PWM- M1
---- TIM9_CH2: PA3=PWM+ M2; GND=PWM- M2
------------------------------------------------------------*/
int append_string_to_buffer(uint8_t *buf, const char *str);
uint8_t ToChar(double value, uint8_t *pBuffer, int NbAfDot);
static void send_Data(void);
void	Fuzzy_ParametersInit(void);
void GPS_StanleyCompute(void);
void Configure(void);
void Init_Parameters(void);
void ConvertCorToByte(double dEast, double dNorht, uint8_t* bEast, uint8_t* bNorth);
void ConvertVelToByte(double dVel, uint8_t* bVel);
void ConvertYawToByte(double dYaw, uint8_t* byaw);
static int nCount = 0;
DC_MOTOR MOTOR1 = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00}; // Right wheel
DC_MOTOR MOTOR2 = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00}; // Left wheel
uint8_t bVMOV[4] = {0x56, 0x4D, 0x4F, 0x56};
uint8_t data[10] = {0};

	int main(){
		uint8_t bEast[5]  = {0};
		uint8_t bNorth[5] = {0};
		uint8_t bYaw[3]   = {0};
		
		Configure();
		Reset_Motor();
		Init_Parameters();
		while(1){
			/*-- Velocity control part --*/
//			if (robot_status.Veh_Auto_Flag == Check_OK){
//				  PID_UpdateSetVel(&MOTOR1,70);
//					PID_UpdateSetVel(&MOTOR2,70);
//			}
			/*-- Fuzzy angle contorl part  --*/
			if (robot_status.Veh_SampleState == Check_OK){
					if (robot_status.Veh_Auto_Flag == Check_OK){
							Fuzzy.dSet_Angle = 60 + Fuzzy.dAngle; // [-180, 180]
							IMU_UpdateFuzzyInput(&Fuzzy);
							Fuzzy.dFuzzy_Out = Defuzzification_Max_Min(Fuzzy.dFuzzy_Error, Fuzzy.dFuzzy_Error_dot);
							if(Fuzzy.dFuzzy_Out >= 0)
							{
							PID_UpdateSetVel(&MOTOR1, (1 - fabs(Fuzzy.dFuzzy_Out)) * Veh.Auto_Velocity);
							PID_UpdateSetVel(&MOTOR2, (1 + fabs(Fuzzy.dFuzzy_Out)) * Veh.Auto_Velocity);
							}
							else
							{
							PID_UpdateSetVel(&MOTOR1, (1 + fabs(Fuzzy.dFuzzy_Out)) * Veh.Auto_Velocity);
							PID_UpdateSetVel(&MOTOR2, (1 - fabs(Fuzzy.dFuzzy_Out)) * Veh.Auto_Velocity);
							}
					}
					robot_status.Veh_SampleState = Check_NOK;
			}
			
			if(robot_status.Vel_Status == Check_OK)
						{
						EncoderRead(&MOTOR1, M1_TIMx, &Timer);
						EncoderRead(&MOTOR2, M2_TIMx, &Timer);
						PID_Compute(&MOTOR1, &Timer);
						PID_Compute(&MOTOR2, &Timer);
						RobotRun(MOTOR1.dOut, MOTOR2.dOut);
						robot_status.Vel_Status = Check_NOK;
						}
		}
}
int append_string_to_buffer(uint8_t *buf, const char *str)
{
    int i = 0;
    while (str[i] != '\0')
    {
        buf[i] = (uint8_t)str[i];
        ++i;
    }
    return i;
}
uint8_t ToChar(double value, uint8_t *pBuffer, int NbAfDot)
{
	uint32_t BefD;
	double AftD;
	uint8_t reverse_buffer[20], reverse_length = 0, index = 0, strleng = 0;
	if(value < 0)
	{
		value = -value;
		pBuffer[index++] = (uint8_t)'-';
	}
	BefD = (uint32_t)value;
	AftD = value - BefD;

	if (BefD == 0) 
	{
		pBuffer[index++] = (uint8_t)'0';
		strleng = index;
	}
	else
	{
		while(BefD != 0)
		{
			reverse_buffer[reverse_length] = (BefD % 10) + 48;
			BefD /= 10;
			reverse_length++;
		}
		strleng = index + reverse_length;
		// take the befD value
		for (int i = 0; i < reverse_length; i++)
		{
			pBuffer[index + i] = reverse_buffer[reverse_length - i - 1];
		}
	}
    /* value is double */
	if(value != (uint32_t)value)
	{
		pBuffer[strleng] = (uint8_t)'.';
		strleng++;
		for (int i = 0; i < NbAfDot; i++)
		{
			AftD *= 10;
			pBuffer[strleng + i] = (uint8_t)AftD + 48;
			AftD = AftD - (uint8_t)AftD;
		}
		strleng += NbAfDot;
	}
	return strleng;
}

static void send_Data(void){
	int index = 0;
	index += append_string_to_buffer(&U1_TxBuffer[index], "$VDATA,");
	index += ToChar(GPS_M8P.dWheelPosX, &U1_TxBuffer[index], 13);
	U1_TxBuffer[index++] = (uint8_t)',';
	index += ToChar(GPS_M8P.dWheelPosY, &U1_TxBuffer[index], 13);
	U1_TxBuffer[index++] = (uint8_t)',';
	index += ToChar(GPS_M8P.dheadingAngle, &U1_TxBuffer[index], 5);
	U1_TxBuffer[index++] = (uint8_t)',';
	index += ToChar(GPS_M8P.refPointIndex, &U1_TxBuffer[index], 1);
	U1_TxBuffer[index++] = (uint8_t)',';
	index += append_string_to_buffer(&U1_TxBuffer[index], "\r\n");	
	U4_SendData(index);
}

void GPS_StanleyCompute(){
		if(GPS_M8P.Goal_Flag == Check_OK)
			return;
		else{
			GPS_StanleyControl(&GPS_M8P, MOTOR1.dCurrent_v, MOTOR2.dCurrent_v);
		}
		if( robot_status.Veh_VelAvoidFlag  == Check_NOK)
		{
			Veh.Auto_Velocity = Veh.Max_Velocity;
		}
//		Veh.Auto_Velocity = Veh.Max_Velocity;
//		GPS_M8P.dDeltaAngle = 30;
		Fuzzy.dSet_Angle = Degree_To_Degree(Fuzzy.dAngle + GPS_M8P.dDeltaAngle); // [-180, 180]
		IMU_UpdateFuzzyInput(&Fuzzy);
		Fuzzy.dFuzzy_Out = Defuzzification_Max_Min(Fuzzy.dFuzzy_Error, Fuzzy.dFuzzy_Error_dot);
		if(Fuzzy.dFuzzy_Out >= 0)
		{
		PID_UpdateSetVel(&MOTOR1, (1 - fabs(Fuzzy.dFuzzy_Out)) * Veh.Auto_Velocity);
		PID_UpdateSetVel(&MOTOR2, (1 + fabs(Fuzzy.dFuzzy_Out)) * Veh.Auto_Velocity);
		}
		else
		{
		PID_UpdateSetVel(&MOTOR1, (1 + fabs(Fuzzy.dFuzzy_Out)) * Veh.Auto_Velocity);
		PID_UpdateSetVel(&MOTOR2, (1 - fabs(Fuzzy.dFuzzy_Out)) * Veh.Auto_Velocity);
		}
		if(GPS_M8P.dGoal_radius <= 0.5)
		{
		GPS_M8P.Goal_Flag = Check_OK;
		PID_UpdateSetVel(&MOTOR1, 0);
		PID_UpdateSetVel(&MOTOR2, 0);
		}
}
	
void Configure(void){
		SysTick_Config(SystemCoreClock / 100); // value for 24-bit down counter register
		USART1_Config(U1_Baudrate);
		USART2_Config(U2_Baudrate);
		UART4_Config(U4_Baudrate);
		USART6_Config(U6_Baudrate);
		Encoder_Config_M1();
		Encoder_Config_M2();
		GPIO_Config();
		PWM_Config();
		InTimer_Config();	
}

void Init_Parameters(void){
		Timer.T = (double)1/20;
		Timer.dvelocity_T = 0.01;
		Veh.Max_Velocity = MPS2RPM(0.5);
		
		UpdateFuzzyCoefficients(&Fuzzy,(double)1/180, (double)1/30, (double)1);
		GPS_ParametersInit(&GPS_M8P);
		PID_ResetPID(&MOTOR1);
		PID_ResetPID(&MOTOR2);
		PID_Set(&MOTOR1,0.66, 1.3, 0.0005);
		PID_Set(&MOTOR2, 0.7, 1.4, 0.0002);
	  Fuzzy_ParametersInit();
		//PID_UpdateSetVel(&MOTOR1,50);
		//PID_UpdateSetVel(&MOTOR2,50);
}
void	Fuzzy_ParametersInit(void)
{
	/*   Input 1 (e = Set_theta - theta)  */
	// NB : -2 - -0.17
	Tramf_Update(&In1_NB,-2,-1,-0.22,-0.17);
	// NS : 0.15 - 0.45
	Trimf_Update(&In1_NS, -0.22, -0.11, 0.001);
	// ZE : 0 - 0.2
	Trimf_Update(&In1_ZE, -0.11, 0, 0.11);
	// PS : 0.15 - 0.45
	Trimf_Update(&In1_PS, 0.001, 0.11, 0.22);
	// PB : 0.4 - 1
	Tramf_Update(&In1_PB,0.17,0.22,1,2);

	/* Input 2 (edot = Set_thetadot - thetadot) */
	// NE : 0.3 - 1
	Tramf_Update(&In2_NE,-2,-1,-0.4,-0.003);
	// ZE : 0 - 0.4
	Trimf_Update(&In2_ZE, -0.4, 0, 0.4);
	// PO : 0.3 - 1
	Tramf_Update(&In2_PO, 0.003, 0.4, 1, 2);
	/* Output value */
	NB = -0.95;
	NM = -0.8;
	NS = -0.4;
	ZE = 0;
	PS = 0.4;
	PM = 0.8;
	PB = 0.95;
}
void ConvertCorToByte(double dEast, double dNorth, uint8_t* bEast, uint8_t* bNorth){
	double North = dNorth - 1191000;
	double East  = dEast - 681000;
	int nNorth   = (int)North;
	int nEast	   = (int)East;
	int nNorthDigit = (North - nNorth)*1000000;
	int nEastDigit = (East - nEast)*1000000;
	//
	bNorth[0] = (nNorth >> 8) & 0xFF;
	bNorth[1] = nNorth & 0xFF;
	bNorth[2] = (nNorthDigit >> 16) & 0xFF;
  bNorth[3] = (nNorthDigit >> 8)  & 0xFF;
  bNorth[4] = nNorthDigit & 0xFF;
	//
	bEast[0] = (nEast >> 8) & 0xFF;
  bEast[1] = nEast & 0xFF;
  bEast[2] = (nEastDigit >> 16) & 0xFF;
  bEast[3] = (nEastDigit >> 8)  & 0xFF;
  bEast[4] = nEastDigit & 0xFF;
}
void ConvertVelToByte(double dVel, uint8_t* bVel){
	int nVel   = (int)dVel;
	int nDigit = (dVel - nVel)*100;
	bVel[0] = nVel & 0xFF;
	bVel[1] = nDigit & 0xFF;
}
void ConvertYawToByte(double dYaw, uint8_t* bYaw){
	memset(bYaw,0,3);
	if (dYaw < 0){
		bYaw[0] = 1;
		dYaw = -dYaw;
	}
	else{
		bYaw[0] = 0;
	}
	int nYaw = (int)dYaw;
	int nDigit = (dYaw - nYaw)*100;
	bYaw[1] = nYaw;
	bYaw[2] = nDigit;
}
