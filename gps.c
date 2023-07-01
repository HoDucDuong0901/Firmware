#include "gps.h"
#include "UART.h"
#include <math.h>
GPS	GPS_M8P;
double GPS_DMS_To_DD(double LL)
{
    double dd, mm;
	dd = (int)(LL / 100); 
	mm = LL - (double)(dd * 100);
	return (dd + mm/60);
}
double GPS_StringToLat(char *inputmessage)
{
	double s1 = 0, s2 = 0;
	int temp = 1000;
	for(int i = 0; i < 4; i++)
	{
		s1 += (inputmessage[i] - 48) * temp;
		temp /= 10;
	}
	temp = 10000;
	for(int i = 5; i < 10; i++)
	{
		s2 += (inputmessage[i] - 48) * temp;
		temp /= 10;
	}
	s2 /= 100000;
	return GPS_DMS_To_DD(s1 + s2);
}
double GPS_StringToLng(char *inputmessage)
{
	double s1 = 0, s2 = 0;
	int temp = 10000;
	for(int i = 0; i < 5; i++)
	{
		s1 += (inputmessage[i] - 48) * temp;
		s2 += (inputmessage[i + 6] - 48) * temp;
		temp /= 10;
	}
	s2 /= 100000;
	return GPS_DMS_To_DD(s1 + s2);
}
void	GetMessageInfo(char *inputmessage, char result[MESSAGE_ROW][MESSAGE_COL], char character){
			int nCol = 0, nRow = 0, nIndex = 0;
			for(int i = 0; i< MESSAGE_ROW; ++i){
				for(int j = 0; j < MESSAGE_COL; j++){
						result[i][j] = 0;	
				}
			}
			while(inputmessage[nIndex] != 0)
			{
				if((inputmessage[nIndex] != 0x0D) && (inputmessage[nIndex + 1] != 0x0A)){
						if(inputmessage[nIndex] != character){
								result[nRow][nCol] = inputmessage[nIndex];
								++nCol;
						}
						else{
								++nRow;
								nCol = 0;
						}
						++nIndex;
				}
				else{
					break;
				}
				
			}
}
enum_Status		GPS_StringCompare(uint8_t* input, char sample[5]){
	for(int nIndex = 0; nIndex < 5; nIndex++){
		if(input[nIndex] != sample[nIndex]){
				return Check_NOK;
			}
		}
		return Check_OK;
}
void GPS_LatLonToUTM(GPS *pgps)
{
	double la, lo, lat, lon, sa, sb, e2, e2cuadrada, c, Huso, S, deltaS, a, epsilon, 
		nu, v, ta, a1, a2, j2, j4, j6, alfa, beta, gama, Bm, xx, yy, dx, dy, coslat;
	la = pgps->dLatitude;
	lo = pgps->dLongtitude;
	sa = 6378137.00000;
	sb = 6356752.314245;
	e2 = pow((sa*sa) - (sb*sb), 0.5) / sb; // do lech cau
	e2cuadrada = e2*e2; // binh phuong do lech cau
	c = (sa*sa) / sb;
	lat = la * (pi / 180); // convert from degree to radian
	lon = lo * (pi / 180); // convert from degree to radian
	coslat = cos(lat);
	Huso = ((int) ((lo / 6) + 31) ); //ZoneNumber
	S = ((Huso * 6) - 183); // //+3 puts origin in middle of zone	
	deltaS = lon - (S * (pi / 180));//compute the UTM Zone from the latitude and longitude
	a = coslat * sin(deltaS); // A = cos(LatRad)*(LongRad-LongOriginRad);
	epsilon = 0.5 * log((1 + a) / (1 - a));
	nu = atan(tan(lat) / cos(deltaS)) - lat;
	v = (c / pow((1 + (e2cuadrada * pow(coslat,2))), 0.5)) * 0.9996;
	ta = (e2cuadrada / 2) * (epsilon*epsilon) * (coslat*coslat); // C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
	a1 = sin(2 * lat);
	a2 = a1 * (coslat*coslat);
	j2 = lat + (a1 / 2);
	j4 = ((3 * j2) + a2) / 4;
	j6 = ((5 * j4) + (a2 * (coslat*coslat))) / 3;
	alfa = ((double)3 / (double)4) * e2cuadrada;
	beta = ((double)5 / (double)3) * (alfa*alfa);
	gama = ((double)35 / (double)27) * (alfa*alfa*alfa);
	Bm = 0.9996 * c * (lat - alfa * j2 + beta * j4 - gama * j6);
	xx = epsilon * v * (1 + (ta / 3)) + 500000;
	yy = nu * v * (1 + ta) + Bm;
	if (yy < 0)
	{
		yy = 9999999 + yy;
	}
	
	dx = xx - pgps->dCurrentPosX; // dx(k) = x(k) - x(k-1)
	dy = yy - pgps->dCurrentPosY; // dy(k) = y(k) - y(k-1)
	/* because CorX = CorY = 0 in the first time */
	if(pgps->dCorX == 0)
	{
		pgps->dCurrentPosX = pgps->dCorX = xx;
		pgps->dCurrentPosY = pgps->dCorY = yy;
		pgps->NewDataAvailable = 1;
	}
	else if(sqrt(dx*dx + dy*dy) < 1) 
	{
		pgps->dCurrentPosX = pgps->dCorX = xx;
		pgps->dCurrentPosY = pgps->dCorY = yy;
		pgps->NewDataAvailable = 1;
	}
}
eGPS_status   GPS_NMEA_Message(GPS *pgps, uint8_t *inputmessage,char result[MESSAGE_ROW][MESSAGE_COL]){
	int nMessageIndex = 0;
	while(inputmessage[nMessageIndex] != '\0' &&  ( nMessageIndex < ROVER_RX_BUFFERSIZE )){
		if(inputmessage[nMessageIndex] == (uint8_t)'$'){
				if( (GPS_StringCompare(&inputmessage[nMessageIndex + 1],"GNGGA")) == Check_OK){
						GetMessageInfo((char *)&inputmessage[nMessageIndex], result, ',');
						GPS_M8P.dLatitude = GPS_StringToLat(&result[2][0]); 
						GPS_M8P.dLongtitude = GPS_StringToLng(&result[4][0]); 
						GPS_LatLonToUTM(&GPS_M8P);
						return GPS_DataValid;
				}
				else{
						return GPS_UnValid;
				}
		}
 }
}
