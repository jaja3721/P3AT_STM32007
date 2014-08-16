#ifndef _CRC_GPS_H
#define _CRC_GPS_H

//#include "main.h"
#define CRC32_POLYNOMIAL 0xEDB88320L
unsigned long CRC32Value(int i);

unsigned long CalculateBlockCRC32(
			unsigned long ulCount, /* Number of bytes in the data block */
			unsigned char *ucBuffer ); /* Data block */


#endif
