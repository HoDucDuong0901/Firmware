#ifndef __GPS_H_
#define __GPS_H_
#include <stdint.h>
#include <stdbool.h>
#define MAX_NUMPOINT_COORDINATE	700
#define    MESSAGE_ROW 15
#define    MESSAGE_COL 20
#define	   pi                               (double)3.14159265358979
extern	char    Message[MESSAGE_ROW][MESSAGE_COL];
typedef enum{
	Invalid        = 0, // No fix/Invalid
	Mode_2D_3D     = 1, // Standard GPS(2D/3D)
	DGNSS          = 2,	// Differential GPS
	Fixed_RTK      = 4,	// RTK Fixed solution
	Float_RTK      = 5,	// RTK float solution
	Dead_Reckoning = 6,	// Estimated (DR) Fix
}enum_GPS_Quality;
typedef enum
{
	Check_NOK = 1,
	Check_OK = 2,
}enum_Status;

typedef enum{
	GPS_DataValid = 1,
	GPS_UnValid   = 2,
}eGPS_status;
typedef struct GPS{
	double dCorX; // east (m) ( is converted from gps-rtk lat/long -> UTM )
	double dCorY; // north (is converted from gps_rtk lat/long -> UTM)
	double dCurrentPosX ; // the current position of x (east)
	double dCurrentPosY ; // the current postition of y (north)
	double dWheelPosX ;
	double dWheelPosY;
	double dGoal_radius; // radius between 	goal and current position
	double defa ; // cross track error
	/* Stanley control variables */
	double dheadingAngle;
	double dThetae;
	double dthetad;
	double dDeltaAngle;
	double dKgain;
	double dKsoft;
	double dStep;
	double dRobotVelocity; // Front wheel (v = (v1 + v2) /2 )
	double dmin;
	/* UTM coordinate Data */
	double dPointX[MAX_NUMPOINT_COORDINATE];
	double dPointY[MAX_NUMPOINT_COORDINATE];
	double dYaw[MAX_NUMPOINT_COORDINATE];
	eGPS_status	GPS_status;
	enum_GPS_Quality 	GPS_quality;
	double 	dLatitude;
	double 	dLongtitude;
	int    NewDataAvailable;
	int 	 	NbOfWayPoints;
	int    	refPointIndex;
	enum_Status         Goal_Flag;
}GPS;
void					GPS_StanleyControl(GPS *pgps, double M1Velocity, double M2Velocity);
void  				GPS_LatLonToUTM(GPS *pgps);  //Get 2 values of lat-lon and update UTM coordiante
void  				GPS_ClearPathBuffer(GPS *pgps);
void  				GPS_UpdatePathYaw(GPS *pgps);
eGPS_status   GPS_NMEA_Message(GPS *pgps, uint8_t *inputmessage,char result[MESSAGE_ROW][MESSAGE_COL]);
void    			GetMessageInfo(char *inputmessage, char result[MESSAGE_ROW][MESSAGE_COL], char character);
enum_Status		GPS_StringCompare(uint8_t* input, char sample[5]);
double 				GetValueFromString(char* string);
double	      GPS_StringToLat(char *inputmessage);
double        GPS_StringToLng(char *inputmessage);
void 					GPS_ParametersInit(GPS* pgps);
extern 				GPS	GPS_M8P;

#endif
