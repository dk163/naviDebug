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
    \ref CUbxPollAidEphAlm implementation

    \brief
    Implementation of \ref CUbxPollAidEphAlm, a class to retrieve Ephemeris and
    Alamanac data from receivers supporting the UBX-AID-* commands
*/
#include "UbxPollAidEphAlm.h"
#include <stdlib.h>
#include <string.h>

unsigned char const CUbxPollAidEphAlm::_aidPoll[_STATE_END_][8] = {
  { 0xB5, 0x62, 0x0B, 0x31, 0x00, 0x00, 0x3C, 0xBF } //!< UBX-AID-ALM poll request
  ,
  { 0xB5, 0x62, 0x0B, 0x30, 0x00, 0x00, 0x3B, 0xBC }
}; //!< UBX-AID-EPH poll request
size_t const CUbxPollAidEphAlm::_expNumMsgs = 32;

/*! Constructor configuring the object. The parameters are forwarded to
    \ref CUbxMsg
*/
CUbxPollAidEphAlm::CUbxPollAidEphAlm(void *context, WRITE_FUNC_p pWrite, size_t maxMsg)
  : CUbxMsg(context, pWrite, maxMsg, false, true), _storage(), _hasAllData(false), _rcvdAnswMsgs(0),
    _state(STATE_EPH)
{
}

/*! Destructor
*/
CUbxPollAidEphAlm::~CUbxPollAidEphAlm() {}

/*! Returns the retrieved data after \ref onNewMsg has returned successfully.

    \param buf             : A pointer which should be set to a copy of the
                             retrieved data. Must not be NULL
    \return                  On success the number of bytes in buf, otherwise
                             a negative value.
*/
ssize_t CUbxPollAidEphAlm::getData(unsigned char **buf)
{
  if (!hasData())
  {
    return -1;
  }
  return _storage.getCopy(buf);
}

/*! Has all data from the receiver been retrieved?

    \return                  true if true, false otherwise
*/
bool CUbxPollAidEphAlm::hasData() { return _hasAllData; }

/*! Implements the purely virtual function of the base
    class
*/
bool CUbxPollAidEphAlm::impl_writeUbxMsg()
{
  return (writeData(_aidPoll[_state], sizeof(_aidPoll[_state])) == sizeof(_aidPoll[_state]));
}

/*! Implements the purely virtual function of the base
    class
*/
TRI_STATE_FUNC_RESULT_t CUbxPollAidEphAlm::impl_isExpectedAnswer(unsigned char const *buf,
                                                                 size_t size)
{
  TRI_STATE_FUNC_RESULT_t result = CALL_AGAIN;
  if (buf && size)
  {
    if (isUbxMessage(buf, size) > 0)
    {
      if (buf[2] == _aidPoll[_state][2] // class is correct
          &&
          buf[3] == _aidPoll[_state][3]) // message id is correct
      {
        resetCounter();
        if (_storage.append(buf, size))
        {
          ++_rcvdAnswMsgs;
          if (_expNumMsgs == _rcvdAnswMsgs) // Received messages
          {
            // Got everything?
            if (_state + 1 == _STATE_END_)
            {
              _hasAllData = true;
              result = SUCCESS;
            }
            else
            {
              // Switch to the next state, reset everything
              // that has to be reset and send out the next
              // poll message
              _state = (STATE_t)((int)_state + 1);
              _rcvdAnswMsgs = 0;
              if (!impl_writeUbxMsg())
              {
                result = FAIL;
              }
            }
          }
        }
        else
        {
          result = FAIL;
        }
      }
    }
  }
  return result;
}

/*! Implements the purely virtual function of the base
    class
*/
void CUbxPollAidEphAlm::impl_clearData()
{
  _storage.clear();
  _rcvdAnswMsgs = 0;
  _hasAllData = false;
  _state = STATE_EPH;
}
