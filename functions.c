#include "functions.h"
double          NB, NM, NS, ZE, PS, PM, PB; // Sugeno output 
trimf           In1_NS, In1_ZE, In1_PS, In2_ZE;
trapmf          In1_NB, In1_PB, In2_NE, In2_PO;
Fuzzy_Parameters	Fuzzy;
double Pi_To_Pi(double angle)
{
	if(angle > pi)
		angle = angle - 2 * pi;
	else if (angle < -pi)
		angle = angle + 2 * pi;

	return angle;
}

void ParsePoint(uint8_t* RxBuffer,GPS* pgps)
{
	uint8_t uTemp[5] = {0};
	pgps->NbOfWayPoints = (RxBuffer[30] << 8) + RxBuffer[31];
	if( pgps->NbOfWayPoints % 5 == 0){
		for(int nIndex = 0; nIndex < 5; nIndex++){
			int npIndex = (pgps->NbOfWayPoints -5) + nIndex;
			memcpy(uTemp,RxBuffer + 32 + nIndex*5,5);
			pgps->dPointX[npIndex]   = (uTemp[0] << 8) + uTemp[1];
			pgps->dPointX[npIndex]  += 681000 + ((double)((uTemp[2] << 16) + (uTemp[3] << 8) + uTemp[4]))/1000000;
			memcpy(uTemp,RxBuffer + 57 + nIndex*5,5);
			pgps->dPointY[npIndex]   = (uTemp[0] << 8) + uTemp[1];
			pgps->dPointY[npIndex]   += 1191000 + ((double)((uTemp[2] << 16) + (uTemp[3] << 8) + uTemp[4]))/1000000;
		}
  }
	else if( pgps->NbOfWayPoints % 5 != 0){
		int npIndex = pgps->NbOfWayPoints % 5;
		for(int nIndex = 0; nIndex < npIndex ; nIndex++){
			memcpy(uTemp,RxBuffer + 32 + nIndex*5,5);
			pgps->dPointX[pgps->NbOfWayPoints - npIndex + nIndex]   = (uTemp[0] << 8) + uTemp[1];
			pgps->dPointX[pgps->NbOfWayPoints - npIndex + nIndex]   += 681000 + ((double)((uTemp[2] << 16) + (uTemp[3] << 8) + uTemp[4]))/1000000;	  
			memcpy(uTemp,RxBuffer + 57 + nIndex*5,5);
			pgps->dPointY[pgps->NbOfWayPoints - npIndex + nIndex]   = (uTemp[0] << 8) + uTemp[1];
			pgps->dPointY[pgps->NbOfWayPoints - npIndex + nIndex]   += 1191000 + ((double)((uTemp[2] << 16) + (uTemp[3] << 8) + uTemp[4]))/1000000;
		}
	}
}	

void ParseYaw(uint8_t* RxBuffer,GPS* pgps){
	uint8_t uTemp[5] = {0};
	pgps->NbOfWayPoints = (RxBuffer[30] << 8) + RxBuffer[31];
		if (pgps->NbOfWayPoints % 5 == 0 ){
		for(int nIndex = 0; nIndex < 5; nIndex++){
			int npIndex = (pgps->NbOfWayPoints-5) + nIndex;
			memcpy(uTemp,RxBuffer + 5 + nIndex*5,5);
			if(uTemp[0] == 0){
				pgps->dYaw[npIndex] = (uTemp[1] + ((double)( (uTemp[2] << 16) + (uTemp[3] << 8) + uTemp[4] ))/1000000); // rad
				}
			else if(uTemp[0] == 1){
				pgps->dYaw[npIndex] = -uTemp[0]*(uTemp[1] + ((double)( (uTemp[2] << 16) + (uTemp[3] << 8) + uTemp[4] ))/1000000); // rad
			  }
		  }
	  }
		else if(pgps->NbOfWayPoints % 5 != 0){
			int nCount = pgps->NbOfWayPoints % 5;
			for(int nIndex = 0; nIndex < nCount - 1 ; nIndex++){
				int npIndex = (pgps->NbOfWayPoints - nCount) + nIndex;
				memcpy(uTemp,RxBuffer + 5 + nIndex*5 ,5);
				if(uTemp[0] == 0){
				pgps->dYaw[npIndex] = (uTemp[1] + ((double)( (uTemp[2] << 16) + (uTemp[3] << 8) + uTemp[4] ))/1000000); // rad
				}
				else if(uTemp[0] == 1){
				pgps->dYaw[npIndex] = -uTemp[0]*(uTemp[1] + ((double)( (uTemp[2] << 16) + (uTemp[3] << 8) + uTemp[4] ))/1000000); // rad
			  }
			}
		}
}
/*-------------------------------------
  ------For fuzzy controler -----------
	-------------------------------------	*/
double Trimf_function(trimf* ptrimf, double x){
	double dResult = 0.0;
	if ( x < ptrimf->L ){
		dResult = 0.0;
	}
	else if( x < ptrimf->C ){
		dResult = (	x - ptrimf->L) / (ptrimf->C - ptrimf->L);
	}
	else if( x < ptrimf->R) {
		dResult = (ptrimf->R - x) / ( ptrimf->R - ptrimf->C);
	}
	else{
		dResult = 0.0;
	}
	return dResult;
}

double Tramf_function(trapmf* ptramf, double x){
	double dResult = 0.0;
	if( x < ptramf->L) {
		dResult = 0.0;
	}
	else if(x < ptramf->C1 ){
		dResult = (x - ptramf->L) / ( ptramf->C1 - ptramf->L);
	}
	else if( x < ptramf->C2){
		dResult = 1.0;
	}
	else if( x < ptramf->R){return
		dResult = (ptramf->R - x) / (ptramf->R - ptramf->C2);
	}
	return dResult;
}

void 	 Trimf_Update(trimf* ptrimf, double L, double C, double R){
	ptrimf->L = L;
	ptrimf->C = C;
	ptrimf->R = R;
}

void   Tramf_Update(trapmf* ptramf, double L, double C1, double C2, double R){
	ptramf->L = L;
	ptramf->C1 = C1;
	ptramf->C2 = C2;
	ptramf->R = R;
}

void 	 Fuzzy_Init(void){
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

double Fuzzy_Max(double *input,int len)
{
	double max;
	max = input[0];
	for(int i = 1; i < len; i++)
	{
		if(max < input[i])
			max = input[i];
	}
	return max;
}

double Defuzzification_Max_Min(double e, double edot)
{
	double pBeta[3], num, den, temp;
	double e_NB, e_NS, e_ZE, e_PS, e_PB, edot_NE, edot_ZE, edot_PO;
	e_NB = Tramf_function(&In1_NB, e);
	e_NS = Trimf_function(&In1_NS, e);
	e_ZE = Trimf_function(&In1_ZE, e);
	e_PS = Trimf_function(&In1_PS, e);
	e_PB = Tramf_function(&In1_PB, e);
	edot_NE = Tramf_function(&In2_NE, edot);
	edot_ZE = Trimf_function(&In2_ZE, edot);
	edot_PO = Tramf_function(&In2_PO, edot);
	//NB and NE is NB
	pBeta[0] = MIN(e_NB, edot_NE);
	num = NB * pBeta[0];
	den = pBeta[0];
	//NS and NE is NM
	//NB and ZE is NM
	pBeta[0] = MIN(e_NS, edot_NE);
	pBeta[1] = MIN(e_NB, edot_ZE);
	temp = Fuzzy_Max(pBeta, 2);
	num += NM * temp;
	den += temp;
	//ZE and NE is NS
	//NS and ZE is NS
	//NB and PO is NS
	pBeta[0] = MIN(e_ZE, edot_NE);
	pBeta[1] = MIN(e_NS, edot_ZE);
	pBeta[2] = MIN(e_NB, edot_PO);
	temp = Fuzzy_Max(pBeta, 3);
	num += NS * temp;
	den += temp;
	//PS and NE is ZE
	//ZE and ZE is ZE
	//NS and PO is ZE
	pBeta[0] = MIN(e_PS, edot_NE);
	pBeta[1] = MIN(e_ZE, edot_ZE);
	pBeta[2] = MIN(e_NS, edot_PO);
	temp = Fuzzy_Max(pBeta, 3);
	num += ZE * temp;
	den += temp;
	//PB and NE is PS
	//PS and ZE is PS
	//ZE and PO is PS
	pBeta[0] = MIN(e_PB, edot_NE);
	pBeta[1] = MIN(e_PS, edot_ZE);
	pBeta[2] = MIN(e_ZE, edot_PO);
	temp = Fuzzy_Max(pBeta,3);
	num += PS * temp;
	den += temp;
	//PB and ZE is PM
	//PS and PO is PM
	pBeta[0] = MIN(e_PB, edot_ZE);
	pBeta[1] = MIN(e_PS, edot_PO);
	temp = Fuzzy_Max(pBeta,2);
	num += PM * temp;
	den += temp;
	//PB and PO is PB
	pBeta[0] = MIN(e_PB, edot_PO);
	num += PB * pBeta[0];
	den += pBeta[0];

	return (den == 0) ? 0 : (num/den);
}

void 	 UpdateFuzzyCoefficients(Fuzzy_Parameters* pFuzzy, double Ke, double Kedot, double Ku){
	pFuzzy->dKe = Ke;
	pFuzzy->dKedot = Kedot;
	pFuzzy->dKu  = Ku;
	pFuzzy->dFuzzy_Error = 0;
	pFuzzy->dFuzzy_Error_dot = 0;
	pFuzzy->dFuzzy_Out = 0;
}

void	IMU_UpdateFuzzyInput(Fuzzy_Parameters *fuzzy){
	fuzzy->dFuzzy_Error = fuzzy->dSet_Angle - fuzzy->dAngle;
	fuzzy->dFuzzy_Error_dot = -(fuzzy->dAngle - fuzzy->dPre_Angle)/Timer.T;
	fuzzy->dPre_Angle = fuzzy->dAngle;
	fuzzy->dPre_Fuzzy_Out = fuzzy->dFuzzy_Out;
	if(fuzzy->dFuzzy_Error > 180 ){
		fuzzy->dFuzzy_Error -= 360;
	}
	else if (fuzzy->dFuzzy_Error < -180){
		fuzzy->dFuzzy_Error += 360;
	}
	fuzzy->dFuzzy_Error *= fuzzy->dKe;
	fuzzy->dFuzzy_Error_dot *= fuzzy->dKedot;
}

void 	 GPS_StanleyControl(GPS* pgps,double v1_rpm, double v2_rpm){
	double dx, dy, d;
	int index = 0, i, upper_bound, lower_bound;
	double v1_mps, v2_mps;
	double L = 0.19, Lf = 0; // L is distance from (Xc, Yc) to the front wheel
	pgps->dheadingAngle = Pi_To_Pi(-Fuzzy.dAngle * (double)pi/180 + pi/2); // heading angle of vehicle [rad] [-pi, pi]
	v1_mps = Wheel_Radius * 2 * pi * v1_rpm / 60; // [m/s]
	v2_mps = Wheel_Radius * 2 * pi * v2_rpm / 60; // [m/s]
	pgps->dRobotVelocity = (v1_mps + v2_mps)/2; // Linear Velocity
	pgps->dRobotVelocity = (pgps->dRobotVelocity < 0) ? -pgps->dRobotVelocity : pgps->dRobotVelocity;
	// Calculate new Pos if there is no new data from GPS
	if(!pgps->NewDataAvailable){
		pgps->dCurrentPosX += pgps->dRobotVelocity*cos(pgps->dheadingAngle)*Timer.T;
		pgps->dCurrentPosY += pgps->dRobotVelocity*sin(pgps->dheadingAngle)*Timer.T;
	}
	// Calculate the front wheel position
	pgps->dWheelPosX = pgps->dCurrentPosX + DISTANCE_BETWEEN_GPS_FRONT_WHEEL*cos(pgps->dheadingAngle);
	pgps->dWheelPosY = pgps->dCurrentPosY + DISTANCE_BETWEEN_GPS_FRONT_WHEEL*sin(pgps->dheadingAngle);
	
	// For searching the nearest point
	if(pgps->refPointIndex == -1){
		lower_bound = 0;
		upper_bound = pgps->NbOfWayPoints;
	}
	else{
		lower_bound = MAX(0, pgps->refPointIndex - SEARCH_OFFSET);
		upper_bound = MIN(pgps->NbOfWayPoints, pgps->refPointIndex + SEARCH_OFFSET);
	}
	for(i = lower_bound; i < upper_bound; ++i)
	{
		dx = pgps->dWheelPosX - pgps->dPointX[i];
		dy = pgps->dWheelPosY - pgps->dPointY[i];
		d  = sqrt(dx*dx + dy*dy);
		if(i == lower_bound) 
		{
			pgps->dmin = d;
			index = i;
		}
		else if(pgps->dmin > d) 
		{
			pgps->dmin = d; 
			index = i;	// position of the minimum value
		}
	}
	Lf = pgps->dKgain * pgps->dRobotVelocity + 0.1;
	while((Lf > L) && (index < pgps->NbOfWayPoints - 1))
	{
		dx = pgps->dWheelPosX - pgps->dPointX[index];
		dy = pgps->dWheelPosY - pgps->dPointY[index];
		L = sqrt(dx*dx + dy*dy);
		index++;
	}
	if( index > pgps->refPointIndex )
		pgps->refPointIndex = index;
	pgps->dGoal_radius = sqrt(pow(pgps->dWheelPosX - pgps->dPointX[pgps->NbOfWayPoints - 1], 2) + 
												pow(pgps->dWheelPosY - pgps->dPointY[pgps->NbOfWayPoints - 1], 2));
	pgps->defa = -((pgps->dWheelPosX - pgps->dPointX[pgps->refPointIndex]) * cos(pgps->dheadingAngle + pi/2) + 
								(pgps->dWheelPosY - pgps->dPointY[pgps->refPointIndex]) * sin(pgps->dheadingAngle + pi/2));
	pgps->dThetae = Pi_To_Pi(pgps->dheadingAngle - pgps->dYaw[pgps->refPointIndex]); // [-pi, pi]
	pgps->dthetad = -atan2( (pgps->dKgain) * (pgps->defa) , (pgps->dRobotVelocity + pgps->dKsoft)); // [-pi, pi]
	pgps->dDeltaAngle  = (pgps->dThetae + pgps->dthetad)*(double)180/pi; 
	if(pgps->dDeltaAngle > 160)
			pgps->dDeltaAngle = 160;
	else if(pgps->dDeltaAngle < -160){
			pgps->dDeltaAngle = -160;
	}	
}

double Degree_To_Degree(double angle)
{
	if(angle > 180)
		angle = angle - 360;
	else if(angle < -180)
		angle = angle + 360;
	return angle;
}

bool 	 StringHeaderCompare(char* s, char* ref, uint8_t size){
	for (uint8_t uIndex = 0; uIndex < size ; uIndex++){
		if (s[uIndex] != ref[uIndex]){
			return false;
		}
	}
	return true;
}

double MPS2RPM(double vel)
{
	return ((vel / (2 * pi * Wheel_Radius)) * 60);
}

