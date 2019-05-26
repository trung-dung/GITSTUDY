/*!
*
*
*
*
*
*
*
*
*
*
*
*
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "nmea.h"
#include "fifo.h"

// DEBUG
#include "main.h"

const char toHex[] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };

//extern nmeaINFO gpsinfo;
void nmea_parse_gpgga(char *nmea, gpgga_t *loc)
{
    char *p = nmea;
	float time;
	unsigned long time_nmea;
	unsigned char hh,mm,ss;
	//printf(nmea);
    p = strchr(p, ',')+1; //skip time 120823.012 120823012
	time = atof(p);
	time_nmea =time;
	hh = time_nmea/10000; 
	time_nmea %= 10000;
	mm = time_nmea/100; 	
	ss = time_nmea%100; 
	
	//m_s = time_nmea;
	DEBUG_D("UTC time(GPGGA): %.03f %02d:%02d:%02d\r\n",time,hh,mm,ss);
    p = strchr(p, ',')+1;
    loc->latitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'N':
            loc->lat = 'N';
            break;
        case 'S':
            loc->lat = 'S';
            break;
        case ',':
            loc->lat = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    loc->longitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'W':
            loc->lon = 'W';
            break;
        case 'E':
            loc->lon = 'E';
            break;
        case ',':
            loc->lon = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    loc->quality = (uint8_t)atoi(p);//Fix quality

    p = strchr(p, ',')+1;
    loc->satellites = (uint8_t)atoi(p);

    p = strchr(p, ',')+1;

    p = strchr(p, ',')+1;
    loc->altitude = atof(p);

	DEBUG_D("\nLat: %f, lon: %f\n", loc->latitude, loc->longitude ); 
}

void nmea_parse_gprmc(char *nmea, gprmc_t *loc)
{
    char *p = nmea;
	float time;
	unsigned long time_nmea;
	unsigned char hh,mm,ss,day,mon,year;
	//printf(nmea);
    p = strchr(p, ',')+1; //skip time	1

	time = atof(p);
	time_nmea =time;
	hh = time_nmea/10000; 
	time_nmea %= 10000;
	mm = time_nmea/100; 	
	ss = time_nmea%100; 
	
	//m_s = time_nmea;
	DEBUG_D("UTC time(GPRMC): %.03f %02d:%02d:%02d\r\n",time,hh,mm,ss);

    p = strchr(p, ',')+1; //skip status	 2

    p = strchr(p, ',')+1;				  //3
    loc->latitude = atof(p);

    p = strchr(p, ',')+1;					 //4
    switch (p[0]) {
        case 'N':
            loc->lat = 'N';
            break;
        case 'S':
            loc->lat = 'S';
            break;
        case ',':
            loc->lat = '\0';
            break;
    }

    p = strchr(p, ',')+1;						//5
    loc->longitude = atof(p);

    p = strchr(p, ',')+1;						   //6
    switch (p[0]) {
        case 'W':
            loc->lon = 'W';
            break;
        case 'E':
            loc->lon = 'E';
            break;
        case ',':
            loc->lon = '\0';
            break;
    }

    p = strchr(p, ',')+1;							  //7	speed
    loc->speed = atof(p);

    p = strchr(p, ',')+1;								 //8 course
    loc->course = atof(p);

	p = strchr(p, ',')+1;								 //9 date
	time = atof(p);
	time_nmea = time;
	day = time_nmea/10000; 
	time_nmea %= 10000;
	mon = time_nmea/100; 	
	year = time_nmea%100; 
	
	//m_s = time_nmea;
	DEBUG_D("UTC date (GPRMC): %.0f %02d-%02d-%02d\r\n",time,day,mon,year);
    if(year < 90)
        year += 100;
    mon -= 1;  
	DEBUG_D("\nLat: %f, lon: %f\n", loc->latitude, loc->longitude );
}

/**
 * Get the message type (GPGGA, GPRMC, etc..)
 *
 * This function filters out also wrong packages (invalid checksum)
 *
 * @param message The NMEA message
 * @return The type of message if it is valid
 */
uint8_t nmea_get_message_type(unsigned char *message)
{
    uint8_t checksum = 0;
    if ((checksum = nmea_valid_checksum(message)) != _EMPTY) {
        return checksum;
    }

    if (strstr((const char *) message, NMEA_GPGGA_STR) != NULL) {
        return NMEA_GPGGA;
    }

    if (strstr((const char *) message, NMEA_GPRMC_STR) != NULL) {
        return NMEA_GPRMC;
    }
    return NMEA_UNKNOWN;
}

uint8_t nmea_valid_checksum(unsigned char * message) {
    uint8_t checksum= (uint8_t)strtol(strchr((const char*) message, '*')+1, NULL, 16);

    char p;
    uint8_t sum = 0;
    ++message;
    while ((p = *message++) != '*') {
        sum ^= p;
    }

    if (sum != checksum) {
        return NMEA_CHECKSUM_ERR;
    }

    return _EMPTY;
}

int _getMessage(Fifo_t *fifo, char* buf, int len)
{
    int unkn = 0;
    int sz = fifo->Size;
    int fr = Free(fifo);
    if (len > sz)
        len = sz;
    while (len > 0)
    {
        // NMEA protocol
        int nmea = _parseNmea(fifo, len);
		if ((nmea != NOT_FOUND) && (unkn > 0))  
            return UNKNOWN | get(fifo, buf, unkn);
        if (nmea == WAIT && fr)                       
            return WAIT;
        if (nmea > 0)                           
            return NMEA | get(fifo, buf, nmea);
        
        // UNKNOWN
        unkn ++;
        len--;
    }
    if (unkn > 0)                      
        return UNKNOWN | get(fifo, buf, unkn); 
    return WAIT;
}

int _parseNmea(Fifo_t *fifo, int len)
{
    int o = 0;
    int c = 0;
    char ch;
    if (++o > len)                      return WAIT;
    if ('$' != FifoPop(fifo))            return NOT_FOUND;
    // this needs to be extended by crc checking 
    for (;;)
    {
        if (++o > len)                  return WAIT;
        ch = FifoPop(fifo);
        if ('*' == ch)                  break; // crc delimiter 
        if (!isprint(ch))               return NOT_FOUND; 
        c ^= ch;
    }
    if (++o > len)                      return WAIT;
    ch = toHex[(c >> 4) & 0xF]; // high nibble
    if (ch != FifoPop(fifo))             return NOT_FOUND;
    if (++o > len)                      return WAIT;
    ch = toHex[(c >> 0) & 0xF]; // low nibble
    if (ch != FifoPop(fifo))             return NOT_FOUND;
    if (++o > len)                      return WAIT;
    if ('\r' != FifoPop(fifo))           return NOT_FOUND;
    if (++o > len)                      return WAIT;
    if ('\n' != FifoPop(fifo))           return NOT_FOUND;
    return o;
}

int _inc(Fifo_t *fifo, int i, int b)
{
	int n = b;
	i += n;
	if (i >= fifo->Size)
		i -= fifo->Size;
	return i;
}

int get(Fifo_t *fifo, char * buf, int n)
{
	int sizeAvailable = Size(fifo);
	int i = 0;
	int c = n;
	if (c < sizeAvailable) sizeAvailable = c;
	memset(buf, 0, sizeAvailable);
	for(i = 0; i < sizeAvailable; i++)
	{
		buf[i] = FifoPop(fifo);
	}
	return n - sizeAvailable;
}

uint8_t processGnss(Fifo_t *fifo)
{
	char c;
	uint8_t cnt = 0, index = 0, res = 0;
	char NMEAString[256];	
	memset(NMEAString, 0, 256); // refresh buffer string
	if(IsFifoEmpty(fifo) == true) // check fifo empty
		return res;
	c = FifoPop(fifo);
	if(c == '$')
	{
	  cnt = 0;
	  NMEAString[index] = c;
	  while(c != '\r' && cnt++ < 200) // chong tran bo dem
	  {
		  c = FifoPop(fifo);
		  if(c != '$')
		  {
			  NMEAString[++index] = c;
		  }
		  else break;
	  }
	  NMEAString[++index] = '\n';
	  NMEAString[++index] = '\0';
	  index = 0;
	  c = '\0';
	  
	  // debug
	  DEBUG_D("\nNMEA: %s", NMEAString);
	  // parse msg and update system infor variables
	  res = parseGPSStr((unsigned char *) NMEAString);
	  HAL_Delay(1000);
	  memset(NMEAString, 0, 256); // clear buffer string
	}	
	return res; 
}

int parseGPSStr(unsigned char *str1)
{
	unsigned char status = 0;
	gpgga_t gpgga;
    gprmc_t gprmc;
    switch (nmea_get_message_type(str1)) 
	{
        case NMEA_GPGGA:
            nmea_parse_gpgga((char *) str1, &gpgga);        
            status |= NMEA_GPGGA;
        break;
        case NMEA_GPRMC:
            nmea_parse_gprmc((char *) str1, &gprmc);
			status |= NMEA_GPRMC;
            break;
    }					 
    return status; // = 0 (not parse); = 1 RMC; = 2 GGA;
}
