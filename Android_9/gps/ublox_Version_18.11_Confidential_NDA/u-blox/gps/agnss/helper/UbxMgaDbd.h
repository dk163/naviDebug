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
    \ref CUbxMgaDbd definition

    \brief
    Definition of \ref CUbxMgaDbd, a class to send MGA-DBD data
    to the receiver. For a class to retrieve MGA-DBD data from
    a receiver please refer to \ref CUbxPollMgaDbd
*/
#ifndef __UBXMGADBD__
#define __UBXMGADBD__
#include "../storage/LockedStorage.h"
#include "UbxMsg.h"
#include "helperFunctions.h"

/*! \class CUbxMgaDbd

    \brief
    Definition of \ref CUbxMgaDbd, a class derived from \ref CUbxMsg,
    which is used to send MGA-DBD data to the receiver.
    For a class to retrieve MGA-DBD data from
    a receiver please refer to \ref CUbxPollMgaDbd
*/
class CUbxMgaDbd : public CUbxMsg
{
public: // Functions
  //! Constructor
  CUbxMgaDbd(void *context, WRITE_FUNC_p pWrite, size_t maxMsg, bool autoClear = true);

  //! Destructor
  //lint -sem(CUbxMgaDbd::impl_clearData,cleanup)
  virtual ~CUbxMgaDbd();

  //! Set the MGA-DBD data to be transferred
  bool setData(unsigned char const *buf, size_t size);

private: // Functions
  /////////////////////////////////////////////////////////////////
  // Start function declarations as required to implemet by base //
  /////////////////////////////////////////////////////////////////
  bool impl_writeUbxMsg();
  TRI_STATE_FUNC_RESULT_t impl_isExpectedAnswer(unsigned char const *buf, size_t size);
  void impl_clearData();
  ////////////////////////////////////////////////////////////////
  // Stop function declarations as required to implemet by base //
  ////////////////////////////////////////////////////////////////

private:                // Variables
  unsigned char *_buf;  //!< Buffer containing the messages to be sent
  BUFC_t *_msgs;        //!< Array of pointers to each message in _buf
  size_t _sentMsgs;     //!< Counts the messages already sent in _msgs
  size_t _sendableMsgs; //!< Number of messages in _msgs

  static const struct UBX_MSG_TYPE _allowed[]; //!< Allowed message types in _buf
};
#endif //__UBXPOLLMGADBD__
