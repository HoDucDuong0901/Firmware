#ifndef __PID_H
#define __PID_H
#include "stdint.h"
typedef struct {
	float dKp,dKi,dKd;
	float pre_dError;
	float dIntergral;
	float pre_dPIDControl;
	float pre2_dError;
	float dPIDControl_term ;
	uint16_t nSampleTuningPID;
}PID_CONTROL_t;
typedef enum{
	NONE = 1,
	SPID,
	CTUN,
	CTUN_RES,
	GPID,
	CSET,
	CRUN,
	CRUN_RES,
	GRMS,
	STOP,
}PROCESS_t;
extern PID_CONTROL_t tPIDControl;
extern PROCESS_t	tProcess;
extern volatile float g_fVref;
extern volatile uint32_t g_nActPulse;
extern volatile uint32_t g_nActPulse_pre;
extern volatile uint32_t g_nCmdVel;
extern volatile uint32_t g_nActVel;
extern volatile float g_fActVelConvert ;
extern volatile uint16_t g_nIndex;
extern volatile float g_dIDError;
extern void PIDReset(PID_CONTROL_t* PID_Ctrl);
extern void PIDInit(PID_CONTROL_t* PID_Ctrl,float dKp,float dKi,float dKd);
extern void PIDTuningSet(PID_CONTROL_t* PID_Ctrl,float dKp,float dKi, float dKd);
extern float PIDCompute(PID_CONTROL_t* PID_Ctrl,float dCmdValue,float dActValue,float dTs);
extern float Convert(float Vel);
#endif
