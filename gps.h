#ifndef __GPS_H_
#define __GPS_H_
#include <stdint.h>
#define MAX_NUMPOINT_COORDINATE	500
#define    MESSAGE_ROW 15
#define    MESSAGE_COL 20
#define	   pi                               (double)3.14159265358979
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
	double dGoas_radius; // radius between 	goal and current position
	double defa ; // cross track error
	/* Stanley control variables */
	double dheadingAngle;
	double dThetae;
	double dthetad;
	double dDeltaAngle;
	double dK;
	double dKsoft;
	double dStep;
	double dRobotVelocity; // Front wheel (v = (v1 + v2) /2 )
	double dmin;
	/* UTM coordinate Data */
	double dPointX[MAX_NUMPOINT_COORDINATE];
	double dPointY[MAX_NUMPOINT_COORDINATE];
	double dYaw[MAX_NUMPOINT_COORDINATE];
	eGPS_status	GPS_status;
	double dLatitude;
	double dLongtitude;
	int    NewDataAvailable;
}GPS;
void					GPS_StanleyControl(GPS *pgps, double M1Velocity, double M2Velocity);
void  				GPS_LatLonToUTM(GPS *pgps);  //Get 2 values of lat-lon and update UTM coordiante
void  				GPS_ClearPathBuffer(GPS *pgps);
void  				GPS_UpdatePathYaw(GPS *pgps);
eGPS_status   GPS_NMEA_Message(GPS *pgps, uint8_t *inputmessage,char result[MESSAGE_ROW][MESSAGE_COL]);
void    			GetMessageInfo(char *inputmessage, char result[MESSAGE_ROW][MESSAGE_COL], char character);
enum_Status		GPS_StringCompare(uint8_t* input, char sample[5]);

#endif