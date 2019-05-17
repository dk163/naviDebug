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
 * $Id: protocolnmea.h 107786 2015-12-09 09:09:00Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/PRODUCTS/libParser/tags/libParser_v1.04/protocolnmea.h $
 *****************************************************************************/

#ifndef __PROTOCOLNMEA_H__
#define __PROTOCOLNMEA_H__

enum {
    NMEA_CHAR_SYNC = 36 /* '$' */,
    NMEA_MAX_SIZE  = 120 /* NMEA standard specifies 82, but receivers can emit more */,
    PUBX_MAX_SIZE  = 512
};
	
extern int NmeaParse(const unsigned char* pBuffer, int iSize);
static int IsValidNmeaGnssType(char t);

#endif //__PROTOCOLNMEA_H__
