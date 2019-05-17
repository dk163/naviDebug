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
    \ref CUbxMgaIniPosLlh implementation

    \brief
    Implementation of \ref CUbxMgaIniPosLlh, a class to send position
    information to the receiver, if it supports MGA-INI-* commands
*/
#include "UbxMgaIniPosLlh.h"
#include <stdlib.h>
#include <string.h>

/*! Constructor configuring the object. The parameters are forwarded to
    \ref CUbxMsg
*/
CUbxMgaIniPosLlh::CUbxMgaIniPosLlh(void *context,
                                   WRITE_FUNC_p pWrite,
                                   size_t maxMsg,
                                   bool autoClear /* = true */)
  : CUbxMsg(context, pWrite, maxMsg, autoClear)
{
  memset(&_pos, 0, sizeof(_pos));
}

/*! Destructor
*/
CUbxMgaIniPosLlh::~CUbxMgaIniPosLlh() {}

/*! Set position to be transferred to the receiver.

    \param pos                : Current position. Must not be NULL
    \return                     true on success, false otherwise
*/
bool CUbxMgaIniPosLlh::setPos(POS_t const *pos)
{
  if (!pos || !isValidPos(pos))
    return false;

  setReadyToSend(true);
  memcpy(&_pos, pos, sizeof(_pos));

  return true;
}

/*! Implements the purely virtual function of the base
    class
*/
bool CUbxMgaIniPosLlh::impl_writeUbxMsg() { return writeUbxMgaIniPos(&_pos); }

/*! Implements the purely virtual function of the base
    class
*/
TRI_STATE_FUNC_RESULT_t CUbxMgaIniPosLlh::impl_isExpectedAnswer(unsigned char const *buf,
                                                                size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = CALL_AGAIN;
  if (buf && size)
  {
    if (isUbxMessage(buf, size) > 0 && size == 16 // Has the size of an MGA-ACK message
        &&
        buf[2] == 0x13 // Is this the MGA class ID?
        &&
        buf[3] == 0x60 // MGA-ACK message
        &&
        buf[9] == 0x40) // UBX-MGA-INI acknowledged
    {
      //ACK? Otherwise NAK
      if (buf[6] == 0x01)
      {
        result = SUCCESS;
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
void CUbxMgaIniPosLlh::impl_clearData() { memset(&_pos, 0, sizeof(_pos)); }

/*!    Send a UBX-MGA-INI-POS_LLH message to the receiver. This implemenation
    of the message support the following aiding:

    \param pos      : The position that should be transmitted
    \return           true if sucessfull, false otherwise
*/
bool CUbxMgaIniPosLlh::writeUbxMgaIniPos(POS_t const *pos)
{
  if (!pos || !isValidPos(pos))
    return false;

  bool result = false;
  MGA_INI_POS_t data;
  if (createMgaIniPos(&data, pos))
  {
    unsigned char *buf = NULL;
    ssize_t bufSize = createUbx(&buf, 0x13, 0x40, &data, sizeof(data));

    if (bufSize >= 0)
    {
      // Write to the receiver
      result = (writeData(buf, bufSize) > 0);
      free(buf);
    }
  }

  return result;
}
