/******************************************************************************
 *
 * Copyright (C) u-blox AG
 * u-blox AG, Thalwil, Switzerland
 *
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice is
 * included in all copies of any software which is or includes a copy or
 * modification of this software and in all copies of the supporting
 * documentation for such software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF
 * THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 ******************************************************************************
 *
 * Project: libParser
 * Purpose: Library providing functions to parse u-blox GNSS receiver messages.
 *
 ******************************************************************************
 * $Id: protocolnmea.cpp 107929 2015-12-10 13:26:48Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/PRODUCTS/libParser/tags/libParser_v1.04/protocolnmea.cpp $
 *****************************************************************************/

#include "protocolnmea.h"
#include "tbox_buffer.h"
//#include "gpsconst.h"
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////

// filter out unresonable values
const double stdDevLimit	= 1000000.0;
const double dopLimit		= 99.9;

///////////////////////////////////////////////////////////////////////////////
int NmeaParse(const unsigned char* pBuffer, int iSize)
{
    // Start
    if (iSize == 0)
    {
        return WAIT;
    }

    if (*pBuffer != NMEA_CHAR_SYNC)
    {
        return NOT_FOUND;
    }

    if (iSize == 1)
    {
        return WAIT;
    }

    // Only default NMEA messages 'G' and PUBX messages 'P' are supported
    int iMaxSize;
    if (*(pBuffer + 1) == 'G')
    {
        if (iSize == 2)
        {
            return WAIT;
        }

        if(!IsValidNmeaGnssType(*(pBuffer + 2)))
        {
            return NOT_FOUND;
        }

        iMaxSize = NMEA_MAX_SIZE;
    }
    else if (*(pBuffer + 1) == 'P')
    {
        iMaxSize = PUBX_MAX_SIZE;
    }
    else
    {
        return NOT_FOUND;
    }

    // NMEA prefix looks good, now look at the payload
    int i;
    for (i = 1; (i < iSize); i ++)
    {
        if (i == iMaxSize)
        {
            // Message still unknown, and max possible NMEA message size reached
            return NOT_FOUND;
        }
        else if (*(pBuffer + i) == '\n')
        {
            // The NMEA message is terminated with a cr lf sequence,
            // since we are tolerant we allow a lf only also
            // if it has a checksum, we have to test it
            int iAsterix = (*(pBuffer + i -1) == '\r') ? i - 4: i - 3;
            if ((iAsterix > 0) && (*(pBuffer + iAsterix) == '*'))
            {
                // the checksum consists of two hex digits (usually in upper case, but we are tolerant)
                unsigned char highNibble = *(pBuffer + iAsterix + 1);
                unsigned char lowNibble  = *(pBuffer + iAsterix + 2);
                highNibble = (highNibble >= 'A' && highNibble <= 'F') ? highNibble - 'A' + 10 :
                             (highNibble >= 'a' && highNibble <= 'f') ? highNibble - 'a' + 10 :
                             (highNibble >= '0' && highNibble <= '9') ? highNibble - '0' : 0xFF ;
                lowNibble  = (lowNibble  >= 'A' && lowNibble  <= 'F') ? lowNibble  - 'A' + 10 :
                             (lowNibble  >= 'a' && lowNibble  <= 'f') ? lowNibble  - 'a' + 10 :
                             (lowNibble  >= '0' && lowNibble  <= '9') ? lowNibble  - '0' : 0xFF ;
                if (lowNibble <= 0xF && highNibble <= 0xF)
                {
                    int calcCRC = 0;
					int j;
                    for (j = 1; j < iAsterix; j++)
                    {
                        calcCRC ^= *(pBuffer + j);
                    }
                    // Get the checksum of NMEA message
                    int msgCRC = (highNibble << 4) | lowNibble;

                    // Compare checksum
                    if (msgCRC != calcCRC)
                    {
						ALOGE("gps CRC ERROR: msgCRC:%d, calcCRC:%d, NMEA:%s", msgCRC, calcCRC, pBuffer);
                    
                        // Checksum doesn't match
                        return i + 1;  //Old: //return NOT_FOUND;  //zhangzhenbang modify for "TBox gps data error" on 20180108
                    }
                }
                else
                {
                    return NOT_FOUND;
                }
            }
			
            return i + 1; // Return NMEA message size
        }
        else if ((!isprint(*(pBuffer + i))) && (!isspace(*(pBuffer + i))))
        {
            // A message should contain printable characters only !
            // we are tolerant and tolerate spaces also
            return NOT_FOUND;
        }
    }
    return WAIT;
}

int IsValidNmeaGnssType(char t)
{
    if ( t == 'P' /* GPS, SBAS, QZSS */
          || t == 'L' /* GLO */
          || t == 'A' /* GAL */
          || t == 'B' /* BDS */
          || t == 'N' /* GNSS */ ) 
	{
      return 1;
  	}
		  
	return 0;
}
