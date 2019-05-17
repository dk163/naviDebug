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
 * $Id: UbxMsg.cpp 103715 2015-10-06 11:18:18Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/helper/UbxMsg.cpp $
 *****************************************************************************/
 
/*! \file
    \ref CUbxMsg partial implementation

    \brief
    Partial implementation of \ref CUbxMsg, an abstract class that is used to
    communicate with a receiver to send or receive UBX messages defined by the
    derived classes.
*/
#include <limits.h>
#include "UbxMsg.h"

/*! Constructor used to configure the base object for the derived classes

    \param context          : A user defined variable that will be passed to
                              the the function pWrite on call
    \param pWrite           : A function pointer to a function that can be
                              called to send data to the receiver
    \param maxMsg           : The maximum number of messages that can be
                              passed to \ref onNewMsg without resulting
                              in a success before FAIL is returned.
    \param autoClear        : If true \ref clearData will be called
                              by the object automatically if \ref onNewMsg
                              returned something else than CALL_AGAIN
    \param readyToSendDef   : Defines the default state of the object.
                              Is it ready to send by default or only
                              after a call to another function first?
*/
CUbxMsg::CUbxMsg(void * context, WRITE_FUNC_p pWrite, size_t maxMsg, bool autoClear /* = true */, bool readyToSendDef /*= false*/)
    : _context(context)
    , _write(pWrite)
    , _dataSent(false)
    , _readyToSend(readyToSendDef)
    , _countMsg(0)
    , _autoClear(autoClear)
    , _readyToSendDef(readyToSendDef)
    , _maxMsg(maxMsg)
{
}

/*! Destructor
*/
CUbxMsg::~CUbxMsg()
{
    _context=NULL;
}

/*! Returns if the object is ready to send data to the receiver or not

    \return                      true if data is stored in the object
                                 and false otherwise
*/
bool CUbxMsg::isReadyToSend()
{
    return _readyToSend;
} 

/*! Returns if the object has already sent data to the receiver

    \return                      true if data has already been sent
                                 false otherwise
*/
bool CUbxMsg::hasSentData()
{
    return _dataSent;
}

/*! Messages retrieved from the receiver can be passed to the object through
    this function. The function will then parse the messages and check
    if any of them finishes the operation started by the call to \ref sendData

    \param buf                : A pointer to a message retrieved from the
                                receiver. May be NULL, in which case a
                                timeout is assumed and the number of messages
                                still allowed to be parsed before an answer
                                must have arrived is decreased by 1.
    \param size               : Number of bytes in buf. May be 0 if buf
                                is NULL
*/ 
TRI_STATE_FUNC_RESULT_t CUbxMsg::onNewMsg(unsigned char const * buf, size_t size)
{
    if(!_readyToSend || !_write)
        return FAIL;

    TRI_STATE_FUNC_RESULT_t result=FAIL;

    ++_countMsg;
    if(_countMsg <= _maxMsg)
    {
        if(!hasSentData())
        {
            if(sendData())
            {
                result=CALL_AGAIN;
            }
        }
        else
        {
            result=impl_isExpectedAnswer(buf, size);
        }
    }
    // Clear data if necessary
    if(result!=CALL_AGAIN && _autoClear)
    {
        clearData();
    }
    return result;
}

/*! Set the readiness of the object to send
    data to the receiver.

    \param readyToSend     : true if the object is ready to send, false
                             otherwise
*/
void CUbxMsg::setReadyToSend(bool readyToSend)
{
    _readyToSend=readyToSend;
}

/*! Helper function, which uses \ref _write to write to the
    receiver and would usually be used by \ref impl_writeUbxMsg

    \param buf             : Pointer to the data that should be
                             written to the receiver
    \param size            : Number of elements in buf
    \return                : Number of bytes written on success,
                             a negative number on failure
*/
ssize_t CUbxMsg::writeData(unsigned char const *buf, size_t size)
{
    if(!_write || !buf || !size || size > SSIZE_MAX)
        return -1;

    return _write(_context, buf, size);
}

/*! Reset the counter for messages to be parsed by \ref onNewMsg
    before an operation-completing message must arrive
*/
void CUbxMsg::resetCounter()
{
    _countMsg=0;
}

/*! Send the stored data to the receiver

    \return                   true on success, false otherwise
*/
bool CUbxMsg::sendData()
{
    if(!isReadyToSend() || _dataSent)
        return false;

    _dataSent=impl_writeUbxMsg();
    return _dataSent;
}

/*! Clear the data stored in this object
*/
void CUbxMsg::clearData()
{
    _dataSent=false;
    _readyToSend=_readyToSendDef;
    resetCounter();
    impl_clearData();
}

