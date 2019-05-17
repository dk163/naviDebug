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
 * $Id: UbxCfgNavX5.cpp 108350 2015-12-16 10:47:43Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/helper/UbxCfgNavX5.cpp $
 *****************************************************************************/
 
/*! \file
    \ref CUbxCfgNavX5 implementation

    \brief
    Implementation of \ref CUbxCfgNavX5, a class to send a UBX-CFG-NAVX5
    message to the receiver.
*/
#include <string.h>
#include <stdlib.h>
#include "UbxCfgNavX5.h"

/*! Constructor configuring the object. The parameters are forwarded to
    \ref CUbxMsg
*/
CUbxCfgNavX5::CUbxCfgNavX5(void * context, WRITE_FUNC_p pWrite, size_t maxMsg, bool autoClear /* = true */)
    : CUbxMsg(context, pWrite, maxMsg, autoClear)
    , _enableAidAck(false)
{
}

/*! Destructor
*/
CUbxCfgNavX5::~CUbxCfgNavX5()
{
}

/*! Configure the object to either enable or disable the acknowledgments
    for aiding messages.

    \param enable          : If true acknowledgments for aiding messages
                             will be enabled. If false they will be disabled
    \return                  true if successful, false otherwise
*/
bool CUbxCfgNavX5::enableAidingAck(bool enable)
{
    setReadyToSend(true);
    _enableAidAck=enable;
    return true;
}

/*! Implements the purely virtual function of the base
    class
*/
TRI_STATE_FUNC_RESULT_t CUbxCfgNavX5::impl_isExpectedAnswer(unsigned char const * buf, size_t size)
{
    TRI_STATE_FUNC_RESULT_t result=CALL_AGAIN;
    if(buf && size)
    {
        // Is this message our ACK?
        if( isUbxMessage(buf, size) > 0
         && size==10       // Has the size of an ACK type message
         && buf[2]==0x05   // Is this the ACK class ID?
         && buf[6]==0x06   // CFG class acknowledged
         && buf[7]==0x23 ) // NAVX5 message acknowledged
        {
            //ACK? Otherwise NAK
            if(buf[3]==0x01)
            {
                result=SUCCESS;
            }
        }
        else
        {
            result=CALL_AGAIN;
        }
    }
    return result;
}

/*! Implements the purely virtual function of the base
    class
*/
void CUbxCfgNavX5::impl_clearData()
{
    _enableAidAck=false;
}

/*! Implements the purely virtual function of the base
    class
*/
bool CUbxCfgNavX5::impl_writeUbxMsg()
{
    return writeUbxCfgNavX5(_enableAidAck);
}

/*!    Send a UBX-CFG-NAVX5 message to the receiver.
    \param enable   : enable Aiding acknowledgments

    \return           true if sucessfull, false otherwise
*/
bool CUbxCfgNavX5::writeUbxCfgNavX5(bool enable)
{
    bool result=false;
    U1 payload[40];
    memset(payload, 0, sizeof(payload));


    // Offset 0 - 1 already set to zero
    payload[2] = 0x00;
    payload[3] = 0x04;
    // Offset 4 - 16 already set to zero
    payload[17] = enable ? 1 : 0;
    // Offset 18 - 36 already set to zero
    // Offset 37 & 39 already set to zero - reserved

    unsigned char *buf=NULL;
    ssize_t bufSize=createUbx(&buf, 0x06, 0x23, payload, sizeof(payload));

    if(bufSize>=0)
    {
        // Write to the receiver
        result=(writeData(buf, bufSize) > 0);
        free(buf);    
    }

    return result;
}

