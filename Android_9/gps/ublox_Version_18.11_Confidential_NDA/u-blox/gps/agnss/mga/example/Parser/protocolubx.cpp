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
 * Purpose: UBX protocol parser
 *
 ******************************************************************************/

#include "protocolubx.h"
#include "parserbuffer.h"

///////////////////////////////////////////////////////////////////////////////
int CProtocolUBX::Parse(unsigned char* pBuffer, int iSize)
{
    if (iSize == 0)
        return CParserBuffer::WAIT;
    if (pBuffer[0] != UBX_CHAR_SYNC0)
        return CParserBuffer::NOT_FOUND;
    if (iSize == 1)
        return CParserBuffer::WAIT;
    if (pBuffer[1] != UBX_CHAR_SYNC1)
        return CParserBuffer::NOT_FOUND;
    if (iSize < 6)
        return CParserBuffer::WAIT;
    U2 iLength = (U2)(  ((U2)pBuffer[4]) +
                        (((U2)pBuffer[5]) << 8));
    // filter out all large messages (with the exception of the tunneling class messages)
    if ((iLength > UBX_MAX_SIZE) &&
        (pBuffer[2] != 0x08/*tunneling class*/))
        return CParserBuffer::NOT_FOUND;
    if (iSize < iLength + 6)
        return CParserBuffer::WAIT;
    // calculate the cksum
    U1 ckA = 0;
    U1 ckB = 0;
    for (int i = 2; i < iLength + 6; i++)
    {
        ckA += pBuffer[i];
        ckB += ckA;
    }
    // check the cksum
    if (iSize < iLength + UBX_FRM_SIZE-1)
        return CParserBuffer::WAIT;
    if (pBuffer[iLength+6] != ckA)
        return CParserBuffer::NOT_FOUND;
    if (iSize < iLength + UBX_FRM_SIZE)
        return CParserBuffer::WAIT;
    if (pBuffer[iLength+7] != ckB)
        return CParserBuffer::NOT_FOUND;
    return iLength + UBX_FRM_SIZE;
}





