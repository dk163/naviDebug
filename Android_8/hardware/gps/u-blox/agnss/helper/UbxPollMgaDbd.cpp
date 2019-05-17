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
 * Project: Android GNSS Driver
 *
 ******************************************************************************
 * $Id: UbxPollMgaDbd.cpp 103715 2015-10-06 11:18:18Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/helper/UbxPollMgaDbd.cpp $
 *****************************************************************************/
 
/*! \file
    \ref CUbxPollMgaDbd implementation

    \brief
    Implementation of \ref CUbxPollMgaDbd, a class to retrieve MGA-DBD
    data from receivers
*/
#include <string.h>
#include <stdlib.h>
#include "UbxPollMgaDbd.h"

/*! Constructor configuring the object. The parameters are forwarded to
    \ref CUbxMsg
*/
CUbxPollMgaDbd::CUbxPollMgaDbd(void * context, WRITE_FUNC_p pWrite, size_t maxMsg)
    : CUbxMsg(context, pWrite, maxMsg, false, true)
    , _storage()
    , _hasAllData(false)
    , _rcvdDbdMsgs(0)
{
}

/*! Destructor
*/
CUbxPollMgaDbd::~CUbxPollMgaDbd()
{
}

/*! Returns the retrieved data after \ref onNewMsg has returned successfully.

    \param buf             : A pointer which should be set to a copy of the
                             retrieved data. Must not be NULL
    \return                  On success the number of bytes in buf, otherwise
                             a negative value.
*/
ssize_t CUbxPollMgaDbd::getData(unsigned char **buf)
{
    if(!hasData())
        return -1;

    return _storage.getCopy(buf);
}

/*! Has all data from the receiver been retrieved?

    \return                  true if true, false otherwise
*/
bool CUbxPollMgaDbd::hasData()
{
    return _hasAllData;
}

/*! Implements the purely virtual function of the base
    class
*/
bool CUbxPollMgaDbd::impl_writeUbxMsg()
{
    unsigned char const mgaDbdPoll[]={ 0xB5, 0x62, 0x13, 0x80, 0x00, 0x00, 0x93, 0xCC };
    return (writeData(mgaDbdPoll, sizeof(mgaDbdPoll))==sizeof(mgaDbdPoll));
}

/*! Implements the purely virtual function of the base
    class
*/
TRI_STATE_FUNC_RESULT_t CUbxPollMgaDbd::impl_isExpectedAnswer(unsigned char const * buf, size_t size)
{
    TRI_STATE_FUNC_RESULT_t result=CALL_AGAIN;
    if(buf && size)
    {
        if( isUbxMessage(buf, size) > 0 )
        {
            if( buf[2] == 0x13   // MGA message
             && buf[3] == 0x80 ) // MGA-DBD message
            {
                if(_storage.append(buf, size))
                {
                    result=CALL_AGAIN;
                    resetCounter();
                    ++_rcvdDbdMsgs;
                }
                else
                {
                    result=FAIL;
                }
            }
            else if( size == 16       // Size of an MGA-ACK message
                  && buf[2] == 0x13   // MGA message
                  && buf[3] == 0x60   // MGA-ACK 
                  && buf[9] == 0x80 ) // Acknowledges MGA-DBD-POLL
            {
                uint32_t expMsgs=buf[10];
                expMsgs|=buf[11] << 8;
                expMsgs|=buf[12] << 16;
                expMsgs|=buf[13] << 24;

                if( buf[6] == 1 // ACK
                 && expMsgs ==  _rcvdDbdMsgs ) // Received message
                {
                    _hasAllData=true;
                    result=SUCCESS;
                }
                else
                {
                    result=FAIL;
                }
            }
        }
    }
    return result;
}

/*! Implements the purely virtual function of the base
    class
*/
void CUbxPollMgaDbd::impl_clearData()
{
    _storage.clear();
    _rcvdDbdMsgs=0;
    _hasAllData=false;
}

