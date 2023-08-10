#include "motor.h"
#include "encoder.h"
VehicleStt 		robot_status;
Time 					Timer; 
Vehicle	        Veh;
void	PID_Compute(DC_MOTOR *motor, Time* pTime){
	motor->dError = motor->dCurrent_set_v - motor->dCurrent_v; // RPM
	double dP = 0.0f,dI = 0.0f,dD = 0.0f;
	dP = motor->dKp*(motor->dError - motor->dError_1);
	dI = motor->dKi*((pTime->dvelocity_T)/2)*(motor->dError + motor->dError_1);
	dD = (motor->dKd/(pTime->dvelocity_T))*(motor->dError_2 - 2*motor->dError_1 + motor->dError);
	motor->dOut = dP + dI + dD + motor->dOut_1;
	//
	motor->dOut = filter(0.08,motor->dOut,motor->dOut_1);
	//
	motor->dError_2 = motor->dError_1;
	motor->dError_1 = motor->dError;
	motor->dOut_1 = motor->dOut;
	if(motor->dOut > MAX_PWM){
		motor->dOut = MAX_PWM;
	}
	else if(motor->dOut < 0 ){
		motor->dOut = 0;
	}
}
void  PID_ResetPID(DC_MOTOR *motor){
	motor->dKd = 0.0f;
	motor->dKi = 0.0f;
	motor->dKp = 0.0f;
	motor->dError = 0.0f;
	motor->dError_1 = 0.0f;
	motor->dError_2 = 0.0f;
	motor->dOut = 0.0f;
	motor->dOut_1 = 0.0f;	
}
void 	PID_Set(DC_MOTOR *motor,double dKp, double dKi, double dKd){
	motor->dKp = dKp;
	motor->dKi = dKi;
	motor->dKd = dKd;
}
void 	Vel_Set(DC_MOTOR *motor,double dVel){
	motor->dCurrent_set_v = dVel;
}
void  PID_UpdateSetVel(DC_MOTOR* motor, double target_v){
	motor->dCurrent_set_v = target_v;
}
void Counter_Reset(DC_MOTOR* motor){
	motor->dOut = 0;
	motor->dOut_1 = 0;
	motor->dCurrent_set_v = 0;
	motor->dCurrent_v = 0;
	motor->uEnc = 0;
	motor->uEnc_pre = 0;
	motor->uDiff_enc = 0;
}
void Set_DirM1(uint8_t dir){
	if( dir == 1){
		GPIO_WriteBit(GPIOC,GPIO_Pin_3,Bit_SET);
	}
	else{
		GPIO_WriteBit(GPIOC,GPIO_Pin_3,Bit_RESET);
	}
}
void Set_DirM2(uint8_t dir){
	if( dir == 1){
		GPIO_WriteBit(GPIOC,GPIO_Pin_4,Bit_SET);
	}
	else{
		GPIO_WriteBit(GPIOC,GPIO_Pin_4,Bit_RESET);
	}
}
void EncoderRead(DC_MOTOR* motor, TIM_TypeDef *TIMx,Time* time){
	motor->uEnc_pre = motor->uEnc;
	motor->uEnc = TIM_GetCounter(TIMx);
	motor->uDiff_enc = motor->uEnc - motor->uEnc_pre;
	if (motor->uDiff_enc > 30000){
		motor->uDiff_enc = motor->uEnc - motor->uEnc_pre - 0xFFFF;
	}
	else if(motor->uDiff_enc < -30000){
		motor->uDiff_enc = motor->uEnc - motor->uEnc_pre + 0xFFFF;
	}
	motor->dCurrent_v = (((double)motor->uDiff_enc / 39400) * 60) / time->dvelocity_T; //39400
}
void RobotRun(double DutyM1,double DutyM2){
	if(DutyM1 > 0){
		Set_DirM1(1);
		//TIM_SetCompare1(TIM9,(uint32_t)((DutyM1 * PWM_PERIOD) / 100));
		PWM_TIMx->CCR1 = (uint32_t)((DutyM1 * 8400) / 100);
	}
	else{
		Set_DirM1(0);
	}
	
	if(DutyM2 > 0){
		Set_DirM2(1);
		//TIM_SetCompare2(TIM9,(uint32_t)((DutyM2 * PWM_PERIOD) / 100));
		PWM_TIMx->CCR2 = (uint32_t)((DutyM2 * PWM_PERIOD) / 100);
	}
	else{
		Set_DirM2(0);
	}
}
double filter(double alpha, double x, double pre_x)
{
	return (1 - alpha)*x + alpha*pre_x;
}
void Reset_Motor()
{
	PID_UpdateSetVel(&MOTOR1, 0);
	PID_UpdateSetVel(&MOTOR2, 0);
	PID_ResetPID(&MOTOR1);
	PID_ResetPID(&MOTOR2);
	Veh.Manual_Velocity = 0.0;
	Veh.Auto_Velocity =   0.0;
	Veh.Max_Velocity =    0.0;
	robot_status.Veh_Auto_Flag = Check_NOK;
	robot_status.Veh_VelAvoidFlag = Check_NOK;
}
