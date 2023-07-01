#include "pid.h"
//#include "tim.h"
volatile float g_fVref = 0.0f;
volatile float g_dIDError = 0;
PID_CONTROL_t tPIDControl ;
PROCESS_t	tProcess = NONE;
volatile uint32_t g_nActPulse = 0;
volatile uint32_t g_nActPulse_pre = 0;
volatile uint32_t g_nCmdVel = 0;
volatile uint32_t g_nActVel = 0;
volatile float g_fActVelConvert = 0;
volatile uint16_t g_nIndex;

void PIDReset(PID_CONTROL_t* PID_Ctrl){
	PID_Ctrl->dKp = 0.0f;
	PID_Ctrl->dKi = 0.0f;	
	PID_Ctrl->dKd = 0.0f;
	PID_Ctrl->dIntergral = 0.0f;
	PID_Ctrl->pre_dError = 0.0f;
	PID_Ctrl->pre2_dError = 0.0f;
	PID_Ctrl->pre_dPIDControl = 0.0f;
	g_dIDError = 0;
}
void PIDInit(PID_CONTROL_t* PID_Ctrl,float dKp,float dKi, float dKd){
	PIDReset(PID_Ctrl);
	PID_Ctrl->dKp = dKp;
	PID_Ctrl->dKi = dKi;	
	PID_Ctrl->dKd = dKd;
	//Set counter
}
void PIDTuningSet(PID_CONTROL_t *PID_Ctrl,float dKp,float dKi,float dKd){
	if(dKp < 0.0f || dKi < 0.0f || dKp < 0.0f){
		return;
	}
	PID_Ctrl->dKp = dKp;
	PID_Ctrl->dKi = dKi;
	PID_Ctrl->dKd = dKd;
}
float PIDCompute(PID_CONTROL_t* PID_Ctrl,float dCmdValue,float dActValue,float dTs){
	float dPIDResult;
	g_dIDError = dCmdValue-dActValue;
	float dP = 0,dI=0,dD =0;
	dP = PID_Ctrl->dKp *(g_dIDError - PID_Ctrl->pre_dError);
	dI = PID_Ctrl->dKi*(dTs/2)*(g_dIDError + PID_Ctrl->pre_dError);
	dD = (PID_Ctrl->dKd/dTs)*(g_dIDError - 2*PID_Ctrl->pre_dError + PID_Ctrl->pre2_dError);
	dPIDResult = dP + dI + dD + PID_Ctrl->pre_dPIDControl;
	PID_Ctrl->pre2_dError = PID_Ctrl->pre_dError;
	PID_Ctrl->pre_dError = g_dIDError;
	PID_Ctrl->pre_dPIDControl = dPIDResult;
	return dPIDResult;
}

