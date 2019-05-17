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
 * $Id: protocolubx.h 107926 2015-12-10 13:15:23Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/PRODUCTS/libParser/tags/libParser_v1.04/protocolubx.h $
 *****************************************************************************/

#ifndef __PROTOCOLUBX_H__
#define __PROTOCOLUBX_H__

#include "ubxStandardTypes.h"
#include "protocol.h"

#ifdef SUPL_ENABLED
#include "ubx_messageDef.h"
#endif

const int UBX_CHAR_SYNC0 = 0xB5;    /* 'µ' */
const int UBX_CHAR_SYNC1 = 0x62;    /* 'b' */
const int UBX_HDR_SIZE = 6;
const int UBX_FTR_SIZE = 2;
const int UBX_FRM_SIZE = UBX_HDR_SIZE + UBX_FTR_SIZE;
const int UBX_MAX_SIZE = 2 * 1024;

class CProtocolUBX : public CProtocol
{
public:
    virtual int Parse(const unsigned char* pBuffer, int iSize);
    virtual void Process(unsigned char* pBuffer, int iSize, CDatabase* pDatabase);
    virtual PROTOCOL_t GetType(void) { return UBX; }

    static int ParseUbx(const unsigned char* pBuffer, int iSize);
    static unsigned int NewMsg(U1 classId, U1 msgId, const void* pPayload, unsigned int iPayloadSize, unsigned char **ppMsg);

protected:
    static void ProcessNavStatus(unsigned char* pBuffer, int iSize, CDatabase* pDatabase);
    static void ProcessNavSvInfo(unsigned char* pBuffer, int iSize, CDatabase* pDatabase);

#ifdef SUPL_ENABLED
    static void ProcessRxmMeas(unsigned char* pBuffer, int iSize, CDatabase* pDatabase);
#endif
};

#endif //__PROTOCOLUBX_H__
