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
    \ref CUbxMgaDbd implementation

    \brief
    Implementation of \ref CUbxMgaDbd, a class to send MGA-DBD data
    to the receiver. For a class to retrieve MGA-DBD data from
    a receiver please refer to \ref CUbxPollMgaDbd
*/
#include "UbxMgaDbd.h"
#include <stdlib.h>
#include <string.h>

const struct UBX_MSG_TYPE CUbxMgaDbd::_allowed[] = {
  { 0x13, 0x80 } // MGA-DBD message type
};

/*! Constructor configuring the object. The parameters are forwarded to
    \ref CUbxMsg
*/
CUbxMgaDbd::CUbxMgaDbd(void *context,
                       WRITE_FUNC_p pWrite,
                       size_t maxMsg,
                       bool autoClear /* = true */)
  : CUbxMsg(context, pWrite, maxMsg, autoClear), _buf(NULL), _msgs(NULL), _sentMsgs(0),
    _sendableMsgs(0)
{
}

/*! Destructor
*/
CUbxMgaDbd::~CUbxMgaDbd()
{
  //lint -e(1506)
  impl_clearData();
}

/*! Pass an array containing concatenated UBX-MGA-DBD messages
    that can be sent later on with \ref sendData for this object

    \param buf                : Pointer to the buffer containing the messages
    \param size               : Number of elements in buf
    \return                     true on success and false otherwise
*/
bool CUbxMgaDbd::setData(unsigned char const *buf, size_t size)
{
  if (!buf || !size)
    return false;

  bool result = false;
  //lint -esym(438, tmpBuf)
  unsigned char *tmpBuf = (unsigned char *)malloc(size);
  if (tmpBuf)
  {
    setReadyToSend(false);
    BUFC_t *msgs = NULL;
    memcpy(tmpBuf, buf, size);
    ssize_t msgsNum =
      verifyUbxMsgsBlock(tmpBuf, size, _allowed, sizeof(_allowed) / sizeof(*_allowed), &msgs);
    if (msgsNum > 0)
    {
      clearData();
      _sendableMsgs = msgsNum;
      _msgs = msgs;
      _buf = tmpBuf;
      setReadyToSend(true);
      result = true;
    }
    else
    {
      free(tmpBuf);
      tmpBuf = NULL;
    }
  }
  return result;
}

/*! Implements the purely virtual function of the base
    class
*/
bool CUbxMgaDbd::impl_writeUbxMsg()
{
  if (!_buf || !_msgs)
  {
    return false;
  }
  return (writeData(_msgs[_sentMsgs].p, _msgs[_sentMsgs].i) == (ssize_t)_msgs[_sentMsgs].i);
}

/*! Implements the purely virtual function of the base
    class
*/
TRI_STATE_FUNC_RESULT_t CUbxMgaDbd::impl_isExpectedAnswer(unsigned char const *buf, size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = CALL_AGAIN;
  if (buf && size)
  {
    if (isUbxMessage(buf, size) > 0)
    {
      if (size == 16         // Size of an MGA-ACK message
          && buf[2] == 0x13  // MGA message
          && buf[3] == 0x60  // MGA-ACK
          && buf[9] == 0x80) // Acknowledges MGA-DBD
      {
        ++_sentMsgs;
        if (_sentMsgs < _sendableMsgs)
        {
          resetCounter();
          impl_writeUbxMsg();
        }
        else
        {
          result = SUCCESS;
        }
      }
    }
  }
  return result;
}

/*! Implements the purely virtual function of the base
    class
*/
void CUbxMgaDbd::impl_clearData()
{
  free(_buf);
  free(_msgs);
  _buf = NULL;
  _msgs = NULL;
  _sentMsgs = 0;
  _sendableMsgs = 0;
}
