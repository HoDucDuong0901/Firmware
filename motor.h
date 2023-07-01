#ifndef	__MOTOR_H_
#define __MOTOR_H_
#include "stdint.h"
#include "stm32f4xx.h" 
#include "stdbool.h"
#define         MAX_PWM                          92
#define         PWM_FREQUENCY      							 20000
#define    			PWM_PERIOD          (168000000 / PWM_FREQUENCY)
// temporary
//
//
typedef struct {
	__IO double dTarget_v;
	__IO double dCurrent_set_v;
	__IO double dCurrent_v;
	__IO double dpre_v;
	__IO double dKp,dKi,dKd;
	__IO double dError; //e(k)
	__IO double dError_1; //e(k-1)
	__IO double dError_2; //e(k-2)
	__IO double dOut;//u(k)
	__IO double dOut_1;//u(k-1)
	__IO uint32_t	uEnc;
	__IO uint32_t uEnc_pre;
	__IO int32_t uDiff_enc;
}DC_MOTOR;
typedef struct {
	bool bveh_run;
	bool bveh_stop;
	
}Status;
typedef struct{
	double      dvelocity_T;
}Time;
extern DC_MOTOR MOTOR1;
extern DC_MOTOR MOTOR2;
extern Status		robot_status;
extern Time sTime;
void	PID_Compute(DC_MOTOR *ipid, Time* pTime);
void  PID_ResetPID(DC_MOTOR *ipid);
void 	PID_Set(DC_MOTOR *motor,double dKp, double dKi, double dKd);
void  PID_UpdateSetVel(DC_MOTOR* motor, double target_v);

void 	Set_DirM1(uint8_t dir);
void 	Set_DirM2(uint8_t dir);
void EncoderRead(DC_MOTOR* motor, TIM_TypeDef *TIMx,Time* time);
void RobotRun(double DutyM1,double DutyM2);
void Counter_Reset(DC_MOTOR* motor);
double filter(double alpha, double x, double pre_x);
float lowpassfilter(uint16_t actVel, uint16_t actVel_1);
#endif
