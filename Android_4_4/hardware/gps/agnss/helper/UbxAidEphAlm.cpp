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
 * $Id: UbxAidEphAlm.cpp 103715 2015-10-06 11:18:18Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/helper/UbxAidEphAlm.cpp $
 *****************************************************************************/
 
/*! \file
    \ref CUbxAidEphAlm implementation

    \brief
    Implementation of \ref CUbxAidEphAlm, a class to send Ephemeris and
    Alamanac data to receivers supporting the UBX-AID-* commands
*/
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include "UbxAidEphAlm.h"

const struct UBX_MSG_TYPE CUbxAidEphAlm::_allowed[]=
{
    { 0x0B, 0x30 }, //AID-ALM
    { 0x0B, 0x31 }  //AID-EPH
};


/*! Constructor configuring the object. The parameters are forwarded to
    \ref CUbxMsg
*/
CUbxAidEphAlm::CUbxAidEphAlm(void * context, WRITE_FUNC_p pWrite, size_t maxMsg, bool autoClear /* = true */)
    : CUbxMsg(context, pWrite, maxMsg, autoClear)
    , _buf(NULL)
    , _msgs(NULL)
    , _sentMsgs(0)
    , _sendableMsgs(0)
{
}

/*! Destructor
*/
CUbxAidEphAlm::~CUbxAidEphAlm()
{
    //lint -e(1506)
    impl_clearData();
}

/*! Pass an array containing concatenated UBX-AID-ALM and UBX-AID-EPH messages
    that can be sent later on with \ref sendData for this object

    \param buf                : Pointer to the buffer containing the messages
    \param size               : Number of elements in buf
    \return                     true on success and false otherwise
*/
bool CUbxAidEphAlm::setData(unsigned char const *buf, size_t size)
{
    if( !buf || !size )
        return false;

    bool result=false;
    //lint -esym(438,tmpBuf)
    unsigned char * tmpBuf=(unsigned char *) malloc(size);
    if(tmpBuf)
    {
        setReadyToSend(false);
        BUFC_t *msgs=NULL;
        memcpy(tmpBuf, buf, size);
        ssize_t msgsNum=verifyUbxMsgsBlock(tmpBuf, size, _allowed, sizeof(_allowed), &msgs);
        if(msgsNum>0)
        {
            clearData();
            _sendableMsgs=msgsNum;
            _msgs=msgs;
            _buf=tmpBuf;
            setReadyToSend(true);
            result=true;
        }
        else
        {
            free(tmpBuf);
            tmpBuf=NULL;
        }
    }
    return result;
}

/*! Implements the purely virtual function of the base
    class
*/
bool CUbxAidEphAlm::impl_writeUbxMsg()
{
    if(!_buf || !_msgs)
    {
        return false;
    }
    return (writeData(_msgs[_sentMsgs].p, _msgs[_sentMsgs].i)==(ssize_t)_msgs[_sentMsgs].i);
}

/*! Implements the purely virtual function of the base
    class
*/
TRI_STATE_FUNC_RESULT_t CUbxAidEphAlm::impl_isExpectedAnswer(unsigned char const * buf, size_t size)
{
    assert(_msgs);
    TRI_STATE_FUNC_RESULT_t result=CALL_AGAIN;
    if(buf && size)
    {
        if(isUbxMessage(buf, size) > 0)
        {
            if( size==10                        // Has the size of an ACK type message
             && buf[2]==0x05                    // Is this the ACK class ID?
             && buf[6]==_msgs[_sentMsgs].p[2]   // class acknowledged
             && buf[7]==_msgs[_sentMsgs].p[3] ) // message acknowledged

            {
                // ACK or NAK. Both is fine
                ++_sentMsgs;
                if(_sentMsgs < _sendableMsgs)
                {
                    resetCounter();
                    impl_writeUbxMsg();
                }
                else
                {
                    result=SUCCESS;
                }
            }
        }
    }
    return result;
}

/*! Implements the purely virtual function of the base
    class
*/
void CUbxAidEphAlm::impl_clearData()
{
    free(_buf);
    free(_msgs);
    _buf=NULL;
    _msgs=NULL;
    _sentMsgs=0;
    _sendableMsgs=0;
}

