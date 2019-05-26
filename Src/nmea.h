#ifndef _NMEA_H_
#define _NMEA_H_

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "fifo.h"

#define _EMPTY 0x00
#define NMEA_GPRMC 0x01
#define NMEA_GPRMC_STR "$GPRMC"
#define NMEA_GPGGA 0x02
#define NMEA_GPGGA_STR "$GPGGA"
#define NMEA_UNKNOWN 0x00
#define _COMPLETED 0x03

#define NMEA_CHECKSUM_ERR 0x80
#define NMEA_MESSAGE_ERR 0xC0

#pragma pack(push,1)

extern const char toHex[];
enum { 
	// getLine Responses
	WAIT      = -1, //!< wait for more incoming data (the start of a message was found, or no data available)
	NOT_FOUND =  0, //!< a parser concluded the the current offset of the pipe doe not contain a valid message

	#define LENGTH(x)   (x & 0x00FFFF)  //!< extract/mask the length
	#define PROTOCOL(x) (x & 0xFF0000)  //!< extract/mask the type

	UNKNOWN   = 0x000000,       //!< message type is unknown 
	UBX       = 0x100000,       //!< message if of protocol NMEA
	NMEA      = 0x200000        //!< message if of protocol UBX
};


struct gpgga {
    // Latitude eg: 4124.8963 (XXYY.ZZKK.. DEG, MIN, SEC.SS)
    double latitude;
    // Latitude eg: N
    char lat;
    // Longitude eg: 08151.6838 (XXXYY.ZZKK.. DEG, MIN, SEC.SS)
    double longitude;
    // Longitude eg: W
    char lon;
    // Quality 0, 1, 2
    uint8_t quality;
    // Number of satellites: 1,2,3,4,5...
    uint8_t satellites;
    // Altitude eg: 280.2 (Meters above mean sea level)
    double altitude;
};
typedef struct gpgga gpgga_t;

struct gprmc {
    double latitude;
    char lat;
    double longitude;
    char lon;
    double speed;
    double course;
};
typedef struct gprmc gprmc_t;

typedef struct _nmeaTIME
{
    int     year;       /**< Years since 1900 */
    int     mon;        /**< Months since January - [0,11] */
    int     day;        /**< Day of the month - [1,31] */
    int     hour;       /**< Hours since midnight - [0,23] */
    int     min;        /**< Minutes after the hour - [0,59] */
    int     sec;        /**< Seconds after the minute - [0,59] */
    int     hsec;       /**< Hundredth part of second - [0,99] */

} nmeaTIME;

typedef struct _nmeaINFO
{
//	int     smask;      /**< Mask specifying types of packages from which data have been obtained */

    nmeaTIME utc;       /**< UTC of position */

    unsigned char     sig;        /**< GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive) */
//    int     fix;        /**< Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D) */

//    double  PDOP;       /**< Position Dilution Of Precision */
//    double  HDOP;       /**< Horizontal Dilution Of Precision */
//    double  VDOP;       /**< Vertical Dilution Of Precision */
    double latitude;
    char lat;
    double longitude;
    char lon;
    int  elv;        /**< Antenna altitude above/below mean sea level (geoid) in meters */
    unsigned short  speed;      /**< Speed over the ground in kilometers/hour */
    unsigned short  direction;  /**< Track angle in degrees True */
	unsigned char sat_inuse;
//    double  declination; /**< Magnetic variation degrees (Easterly var. subtracts from true course) */

//    nmeaSATINFO satinfo; /**< Satellites information */

} nmeaINFO;
#pragma pack(pop)

uint8_t nmea_get_message_type(unsigned char *);
uint8_t nmea_valid_checksum(unsigned char *);
void nmea_parse_gpgga(char *, gpgga_t *);
void nmea_parse_gprmc(char *, gprmc_t *);
int _parseNmea(Fifo_t *fifo, int len);
int _getMessage(Fifo_t *fifo, char* buf, int len);
int _inc(Fifo_t *fifo, int i, int b);

/*! get elements from the buffered pipe
	\param p the elements extracted
	\param n the maximum number elements to extract
	\param t set to true if blocking, false otherwise
	\return number elements extracted
du lieu lay duoc ghi vao buffer
*/
int get(Fifo_t *fifo, char * buf, int n);

// bien dau vao: bo dem du lieu chua ban tin
uint8_t processGnss(Fifo_t *fifo);

// phan tich cu phap chuoi gps
int parseGPSStr(unsigned char *str1);


#endif

