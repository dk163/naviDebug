/*******************************************************************************
 *
 * Copyright (C) u-blox AG
 * u-blox AG, Thalwil, Switzerland
 *
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice
 * is included in all copies of any software which is or includes a copy
 * or modification of this software and in all copies of the supporting
 * documentation for such software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
 * OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 ******************************************************************************
 *
 * Project: libMGA
 * Purpose: NMEA protocol parser
 *
 ******************************************************************************/

#include "protocolnmea.h"
#include "parserbuffer.h"
#include <ctype.h>


int CProtocolNMEA::Parse(unsigned char* pBuffer, int iSize)
{
    // Start
    if (iSize == 0)
        return CParserBuffer::WAIT;
    if (pBuffer[0] != NMEA_CHAR_SYNC)
        return CParserBuffer::NOT_FOUND;
    if (iSize == 1)
        return CParserBuffer::WAIT;
    int iMaxSize;
    if (pBuffer[1] == 'G')
    {
        if (iSize == 2)
            return CParserBuffer::WAIT;
        if ((pBuffer[2] != 'P' /* GPS */ ) && (pBuffer[2] != 'L' /* GLO */  ) &&
            (pBuffer[2] != 'A' /* GAL */ ) && (pBuffer[2] != 'N' /* GNSS */ ) &&
            (pBuffer[2] != 'B' /* BDS */ ))
            return CParserBuffer::NOT_FOUND;
        iMaxSize = NMEA_MAX_SIZE;
    }
    else
    {
        if (pBuffer[1] != 'P')
            return CParserBuffer::NOT_FOUND;
        iMaxSize = PUBX_MAX_SIZE;
    }
    // Payload
    for (int i = 1; (i < iSize); i ++)
    {
        if (i == iMaxSize)
            return CParserBuffer::NOT_FOUND;
        else if (pBuffer[i] == '\n')
        {
            // the nmea message is terminated with a cr lf squence,
            // since we are toleran we allow a lf only also
            // if it has a checksum, we have to test it
            int iAsterix = (pBuffer[i-1] == '\r') ? i - 4: i - 3;
            if ((iAsterix > 0) && (pBuffer[iAsterix] == '*'))
            {
                // the checksum consists of two hex digits (usually in upper case, but we are tolerant)
                unsigned char highNibble = pBuffer[iAsterix + 1];
                unsigned char lowNibble  = pBuffer[iAsterix + 2];
                highNibble = (highNibble >= 'A' && highNibble <= 'F') ? highNibble - 'A' + 10 :
                             (highNibble >= 'a' && highNibble <= 'f') ? highNibble - 'a' + 10 :
                             (highNibble >= '0' && highNibble <= '9') ? highNibble - '0' : 0xFF ;
                lowNibble  = (lowNibble  >= 'A' && lowNibble  <= 'F') ? lowNibble  - 'A' + 10 :
                             (lowNibble  >= 'a' && lowNibble  <= 'f') ? lowNibble  - 'a' + 10 :
                             (lowNibble  >= '0' && lowNibble  <= '9') ? lowNibble  - '0' : 0xFF ;
                if (lowNibble <= 0xF && highNibble <= 0xF)
                {
                    int calcCRC = 0;
                    for (int j = 1; j < iAsterix; j++)
                        calcCRC ^= pBuffer[j];
                    // Checksumme der NMEA-Message holen
                    int msgCRC = (highNibble << 4) | lowNibble;
                    // Checksumme vergleichen
                    if (msgCRC != calcCRC)
                        return CParserBuffer::NOT_FOUND;
                }
                else
                    return CParserBuffer::NOT_FOUND;
            }
            return i+1;
        }
        else if ((!isprint(pBuffer[i])) && (!isspace(pBuffer[i])))
        {
            // a message should contain printable characters only !
            // we are tolerant and tolerate spaces also
            return CParserBuffer::NOT_FOUND;
        }
    }
    return CParserBuffer::WAIT;
}

