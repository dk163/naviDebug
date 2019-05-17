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
    \ref CUbxMgaIniTimeUtc implementation

    \brief
    Implementation of \ref CUbxMgaIniTimeUtc, a class to send time
    information to the receiver, if it supports UBX-MGA-INI* commands
*/
#include "UbxMgaIniTimeUtc.h"
#include <stdlib.h>
#include <string.h>

/*! Constructor configuring the object. The parameters are forwarded to
    \ref CUbxMsg
*/
CUbxMgaIniTimeUtc::CUbxMgaIniTimeUtc(void *context,
                                     WRITE_FUNC_p pWrite,
                                     size_t maxMsg,
                                     bool autoClear /* = true */)
  : CUbxMsg(context, pWrite, maxMsg, autoClear)
{
  memset(&_accTime, 0, sizeof(_accTime));
}

/*! Destructor
*/
CUbxMgaIniTimeUtc::~CUbxMgaIniTimeUtc() {}

/*! Set the time to be transferred to the receiver.

    \param accTime            : Current Time information. Must not be NULL.
    \return                     true on success, false otherwise
*/
bool CUbxMgaIniTimeUtc::setTime(ACCTIME_t const *accTime)
{
  if (!isValidAccTime(accTime))
    return false;

  setReadyToSend(true);
  memcpy(&_accTime, accTime, sizeof(_accTime));

  deductLeapSec(&_accTime);

  return true;
}

/*! Implements the purely virtual function of the base
    class
*/
bool CUbxMgaIniTimeUtc::impl_writeUbxMsg() { return writeUbxMgaIniTimeUtc(&_accTime); }

/*! Implements the purely virtual function of the base
    class
*/
TRI_STATE_FUNC_RESULT_t CUbxMgaIniTimeUtc::impl_isExpectedAnswer(unsigned char const *buf,
                                                                 size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = CALL_AGAIN;
  if (buf && size)
  {
    if (isUbxMessage(buf, size) > 0 && size == 16 // Has the size of an MGA-ACK message
        &&
        buf[2] == 0x13 // Is this the MGA class ID?
        &&
        buf[3] == 0x60 // MGA-ACK AID message
        &&
        buf[9] == 0x40) // UBX-MGA-INI acknowledged
    {
      //ACK? Otherwise NAK
      if (buf[6] == 0x01)
        result = SUCCESS;
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
void CUbxMgaIniTimeUtc::impl_clearData() { memset(&_accTime, 0, sizeof(_accTime)); }

/*! Send a UBX-MGA-INI-TIME_UTC message to the receiver.

    \param accTime            : The time that should be written to the receiver
    \return                     true if sucessful, false otherwise
*/
bool CUbxMgaIniTimeUtc::writeUbxMgaIniTimeUtc(ACCTIME_t const *accTime)
{
  if (!accTime || !isValidAccTime(accTime))
    return false;

  bool result = false;
  MGA_INI_TIME_UTC_t data;
  if (createMgaIniTimeUtc(&data, accTime))
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
