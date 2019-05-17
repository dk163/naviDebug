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
#ifndef __PROTOCOLNMEA_H__
#define __PROTOCOLNMEA_H__

#include "protocol.h"

class CProtocolNMEA : public CProtocol
{
public:
    enum {
        NMEA_CHAR_SYNC = 36 /* '$' */,
        NMEA_MAX_SIZE  = 82 /* this is the limit of the NMEA standard */,
        PUBX_MAX_SIZE  = 512
    };

    int Parse(unsigned char* pBuffer, int iSize);
};

#endif //__PROTOCOLNMEA_H__
