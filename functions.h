#ifndef __FUNCTIONS_H_
#define __FUNCTIONS_H_
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "gps.h"
#include "motor.h"
/* For getting data points*/
#define	           Wheel_Radius                     0.085
#define            DISTANCE_BETWEEN_GPS_FRONT_WHEEL 0.36
#define            SEARCH_OFFSET                    5          // for nearest point searching
void ParsePoint(uint8_t* RxBuffer,GPS* pgps);
void ParseYaw(uint8_t*   RxBuffer,GPS* pgps);


/*For Fuzzy logic variables and functions */
#define					MAX(a,	b)		((a) < (b)) ? (b) : (a)
#define 				MIN(a, b)			((a) < (b)) ? (a) : (b)
#define					PROD(a, b)    ((a)*(b))
typedef struct trimf{
	double L;
	double C;
	double R;
}trimf;
typedef struct trapmf{
	double L;
	double C1;
	double C2;
	double R;
}trapmf;
typedef struct fuzzy{
		/*Current angle and Set angle */
		double dAngle;
		double dSet_Angle;
		double dPre_Angle;
		/*Fuzzy input and output */
		double dFuzzy_Out;
		double dPre_Fuzzy_Out;
		double dFuzzy_Error;
		double dFuzzy_Error_dot;
		/* Fuzzy parameters */
		double dKe;
		double dKedot;
		double dKu;
}Fuzzy_Parameters;
extern Fuzzy_Parameters Fuzzy;
double Trimf_function(trimf* ptrimf, double x);
double Tramf_function(trapmf* ptramf, double x);
void 	 Trimf_Update(trimf* ptrimf, double L, double C, double R);
void   Tramf_Update(trapmf* ptramf, double L, double C1, double C2, double R);
void 	 Fuzzy_Init(void);
double Fuzzy_Max(double *input,int len);
void 	 UpdateFuzzyCoefficients(Fuzzy_Parameters* pFuzzy, double Ke, double Kedot, double Ku);
/*----------------------------For Stanley--------------------*/
void 	 GPS_StanleyControl(GPS* pgps,double v1_rpm, double v2_rpm);
double Pi_To_Pi(double angle);
double Degree_To_Degree(double angle);
void	 IMU_UpdateFuzzyInput(Fuzzy_Parameters *pimu);
bool 	 StringHeaderCompare(char* s, char* ref, uint8_t size); 
double Defuzzification_Max_Min(double e, double edot);
double MPS2RPM(double vel);

// Variable 
extern double          NB, NM, NS, ZE, PS, PM, PB; // Sugeno output 
extern trimf           In1_NS, In1_ZE, In1_PS, In2_ZE;
extern trapmf          In1_NB, In1_PB, In2_NE, In2_PO;
#endif
