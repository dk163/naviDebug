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
 * $Id$
 * $HeadURL$
 *****************************************************************************/

/*! \file
    \ref CUbxAidIni Implementation

    \brief
    Implementation of \ref CUbxAidIni, a class derived from \ref CUbxMsg,
    which is used to send time and position information to a receiver
    supporting UBX-AID-* messages.
*/
#include "UbxAidIni.h"
#include <stdlib.h>
#include <string.h>

/*! Constructor configuring the object. The parameters are forwarded to
    \ref CUbxMsg
*/
CUbxAidIni::CUbxAidIni(void *context,
                       WRITE_FUNC_p pWrite,
                       size_t maxMsg,
                       bool autoClear /* = true */)
  : CUbxMsg(context, pWrite, maxMsg, autoClear)
{
  memset(&_accTime, 0, sizeof(_accTime));
  memset(&_pos, 0, sizeof(_pos));
}

/*! Destructor
*/
CUbxAidIni::~CUbxAidIni() {}

/*! Set position and/or time to be transferred to the receiver. Time information
    must be passed.

    \param accTime            : Current Time information. Must not be NULL.
    \param pos                : Current position. May be NULL
    \return                     true on success, false otherwise
*/
bool CUbxAidIni::setPosTime(ACCTIME_t const *accTime, POS_t const *pos /*= NULL*/)
{
  if (!accTime || !isValidAccTime(accTime) || (pos && !isValidPos(pos)))
    return false;

  memcpy(&_accTime, accTime, sizeof(_accTime));
  if (pos) // Position and valid Time
  {
    memcpy(&_pos, pos, sizeof(_pos));
  }
  setReadyToSend(true);

  return true;
}

/*! Implements the purely virtual function of the base
    class
*/
bool CUbxAidIni::impl_writeUbxMsg() { return writeUbxAidIni(&_accTime, _pos.valid ? &_pos : NULL); }

/*! Implements the purely virtual function of the base
    class
*/
TRI_STATE_FUNC_RESULT_t CUbxAidIni::impl_isExpectedAnswer(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = CALL_AGAIN;
  if (buf && size)
  {
    if (isUbxMessage(buf, size) > 0 && size == 10 // Has the size of an ACK type message
        &&
        buf[2] == 0x05 // Is this the ACK class ID?
        &&
        buf[6] == 0x0B // AID message acknowledged
        &&
        buf[7] == 0x01) // INI message acknowledged
    {
      //ACK? Otherwise NAK
      if (buf[3] == 0x01)
      {
        result = SUCCESS;
      }
      else
      {
        result = FAIL;
      }
    }
    else
    {
      result = CALL_AGAIN;
    }
  }
  return result;
}

/*! Implements the purely virtual function of the base
    class
*/
void CUbxAidIni::impl_clearData()
{
  memset(&_accTime, 0, sizeof(_accTime));
  memset(&_pos, 0, sizeof(_pos));
}

/*!    Send a UBX-AID-INI message to the receiver. This implemenation of the message
    support the following aiding:
    \param accTime   : The time that should be used for aiding. Must not be NULL
    \param pos       : The position that should be used for aiding.
                       May be NULL.
    \return            true if sucessfull, false otherwise
*/
bool CUbxAidIni::writeUbxAidIni(ACCTIME_t const *accTime, POS_t const *pos)
{
  bool result = false;
  GPS_UBX_AID_INI_U5__t Payload;
  // only need to do something if the is at least one part of the information available
  if (createUbxAidIni(&Payload, accTime, pos))
  {
    if (Payload.flags != 0)
    {
      unsigned char *ubxmsg = NULL;
      ssize_t ubxsize = createUbx(&ubxmsg, 0x0B, 0x01, &Payload, sizeof(Payload));
      if (ubxsize > 0)
      {
        result = (writeData(ubxmsg, ubxsize) > 0);
        free(ubxmsg);
      }
    }
  }
  return result;
}
