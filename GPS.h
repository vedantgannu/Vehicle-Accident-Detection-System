#ifndef _GPS_H_
#define _GPS_H_

#include <stdint.h>

//##################################################################################################################
typedef struct
{
	char 			status;//Current data validity
	uint8_t			UTC_Hour;//UTC hours (24 hour)
	uint8_t			UTC_Min;//UTC Minutes
	uint8_t			UTC_Sec;//UTC Seconds
	uint16_t		UTC_MicroSec;//UTC Microseconds (Not used for this GPS module)

	float				Latitude;//Degrees and minutes of Latitude
	double			LatitudeDecimal;//Degrees format Latitude
	char				NS_Indicator;//North or South
	float				Longitude;//Degrees and minutes of Longitude
	double			LongitudeDecimal;//Degrees format Longitude
	char				EW_Indicator;//East or West

	uint8_t			UTC_Month;//UTC Month
	uint8_t			UTC_Day;//UTC Day
	uint8_t			UTC_Year;//UTC Year
	//char				CheckSum[2];

}GPRMC_t;


typedef struct 
{
	uint8_t		rxBuffer[512];//Used for storing NMEA sentences
	uint16_t	rxIndex;//Maintain position in rxBuffer
	uint8_t		rxTmp;//Temporary storage of GPS byte
	uint32_t	LastTime;//milliseconds since startup of when byte was received
	
	GPRMC_t		GPRMC;//Struct for GPRMC NMEA sentence information
	
}GPS_t;

extern GPS_t GPS;//Default constructor used
//##################################################################################################################
void	GPS_Init(void);
void	GPS_CallBack(void);
void	GPS_Process(void);
//##################################################################################################################

#endif
