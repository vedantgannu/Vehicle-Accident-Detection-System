#include "GPSConfig.h"
#include "GPS.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

GPS_t GPS;
//##################################################################################################################

double convertDegMinToDecDeg (float degMin, char direction)
{//Used for converting Lat and Long in degrees and minutes format to only degrees
  double min = 0.0;
  double decDeg = 0.0;

  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);//Floating point remainder representing the minutes portion

  //rebuild coordinates in decimal degrees
  decDeg = (int) ( degMin / 100 );
  decDeg = decDeg + ( min / 60 );
  if (direction == 'S' || direction == 'W'){
	  decDeg *= -1;
  }
  return decDeg;
}

//##################################################################################################################
void GPS_Init(void)
{//Used for initializing the GPS
	GPS.GPRMC.status = 'V';
	GPS.rxIndex=0;
	HAL_UART_Receive_IT(&_GPS_USART,&GPS.rxTmp,1);	
}
//##################################################################################################################
void GPS_CallBack(void)
{//Called in the USART6_IRQ_Handler
	GPS.LastTime=HAL_GetTick();//Store when last time that callback was called
	if(GPS.rxIndex < sizeof(GPS.rxBuffer)-2)//Store received byte in rxBuffer and increment index for next byte
	{
		GPS.rxBuffer[GPS.rxIndex] = GPS.rxTmp;
		GPS.rxIndex++;
	}	
	HAL_UART_Receive_IT(&_GPS_USART,&GPS.rxTmp,1);//Set up receive for next byte
}
//##################################################################################################################
void GPS_Process(void)
{
	if( (HAL_GetTick()-GPS.LastTime>50) && (GPS.rxIndex>0))
	{//Once all NMEA sentences have been constructed for a single UART Receive call
		char *str;
		#if (_GPS_DEBUG==1)//Used for printing out all received NMEA statements
		printf("%s\r\n",GPS.rxBuffer);
		#endif
		str=strstr((char*)GPS.rxBuffer,"$GPRMC,");//Parsing for the $GPRMC NMEA sentence
		if(str!=NULL){//If GPRMC sentence exists
			char* UTC_time;
			char* status;
			char* Latitude;
			char* N_or_S;
			char* Longitude;
			char* E_or_W;
			char* UTC_Date;
			//memset(&GPS.GPRMC,0,sizeof(GPS.GPRMC));
			//$GPGGA,015805.00,4104.71395,N,07332.05869,W,1,10,0.86,42.1,M,-34.2,M,,*58
			strsep(&str, ",");//Omit the $GPRMC string
			UTC_time = strsep(&str, ",");//The UTC Time
			status = strsep(&str, ",");//The Data Status
			Latitude = strsep(&str, ",");//Latitude
			N_or_S = strsep(&str, ",");//North or South
			Longitude = strsep(&str, ",");//Longitude
			E_or_W = strsep(&str, ",");//East or West
			strsep(&str, ",");//Omit Speed over ground
			strsep(&str, ",");//Omit Course over ground
			UTC_Date = strsep(&str, ",");//UTC Date of position fix, ddmmyy
			//Check if valid data did come through - Data that has all desired fields
			if (status[0] == 'V'){//If the GRMC data is invalid
				GPS.GPRMC.status = 'V';//Set status flag to indicate invalid data
				printf("GPS Standby\r\n");//Indicate Invalid data and don't change previous data
			}
			else {//If the GRMC data is valid
				memset(&GPS.GPRMC,0,sizeof(GPS.GPRMC));//Delete previous data and store new data
				GPS.GPRMC.status = status[0];//Set GPS data status
				GPS.GPRMC.NS_Indicator = N_or_S[0];
				GPS.GPRMC.EW_Indicator = E_or_W[0];
				char time[3];
				//Copy the Hours, Minutes, Seconds fields from UTC time
				strncpy(time, UTC_time, 2);time[2] = '\0';
				GPS.GPRMC.UTC_Hour = (uint8_t)atoi(time);//Store the UTC time Hours
				strncpy(time, UTC_time + 2, 2);
				GPS.GPRMC.UTC_Min = (uint8_t)atoi(time);//Store the UTC time Minutes
				strncpy(time, UTC_time + 4, 2);
				GPS.GPRMC.UTC_Sec = (uint8_t)atoi(time);//Store the UTC time Seconds
				//ddmmyy - input UTC date format
				char date[3];
				strncpy(date, UTC_Date, 2);date[2] = '\0';
				GPS.GPRMC.UTC_Day = (uint8_t)atoi(date);//Store the UTC Day
				strncpy(date, UTC_Date + 2, 2);
				GPS.GPRMC.UTC_Month = (uint8_t)atoi(date);//Store the UTC Month
				strncpy(date, UTC_Date + 4, 2);
				GPS.GPRMC.UTC_Year = (uint8_t)atoi(date);//Store the UTC Year
				GPS.GPRMC.Latitude = strtof(Latitude, NULL);//Store the dddmm.mmmm Latitude data
				GPS.GPRMC.Longitude = strtof(Longitude, NULL);//Store the dddmm.mmmm Longitude data
				GPS.GPRMC.LatitudeDecimal=convertDegMinToDecDeg(GPS.GPRMC.Latitude, N_or_S[0]);
				GPS.GPRMC.LongitudeDecimal=convertDegMinToDecDeg(GPS.GPRMC.Longitude, E_or_W[0]);
				//printf("\"%d:%d:%d (UTC)\", \"%d-%d-%d (UTC)\", \"%f %c\", \"%f %c\"\r\n",GPS.GPRMC.UTC_Hour, GPS.GPRMC.UTC_Min, GPS.GPRMC.UTC_Sec, GPS.GPRMC.UTC_Month, GPS.GPRMC.UTC_Day, GPS.GPRMC.UTC_Year, GPS.GPRMC.LatitudeDecimal, GPS.GPRMC.NS_Indicator, GPS.GPRMC.LongitudeDecimal, GPS.GPRMC.EW_Indicator);
			}
		}
		memset(GPS.rxBuffer,0,sizeof(GPS.rxBuffer));//Reset the NMEA receive buffer
		GPS.rxIndex=0;//Reset index to beginning
	}
	HAL_UART_Receive_IT(&_GPS_USART,&GPS.rxTmp,1);//Initiate a byte receive from GPS
}
//##################################################################################################################
