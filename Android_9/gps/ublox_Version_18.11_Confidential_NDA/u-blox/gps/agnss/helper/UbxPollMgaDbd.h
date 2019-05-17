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
    \ref CUbxPollMgaDbd definition

    \brief
    Definition of \ref CUbxPollMgaDbd, a class to retrieve MGA-DBD
    data from receivers
*/
#ifndef __UBXPOLLMGADBD__
#define __UBXPOLLMGADBD__
#include "../storage/LockedStorage.h"
#include "UbxMsg.h"
#include "helperFunctions.h"

/*! \class CUbxPollMgaDbd

    \brief
    Definition of \ref CUbxPollMgaDbd, a class derived from \ref CUbxMsg,
    which is used to retrieve MGA-DBD data from a receiver.
    Please refer to \ref CUbxMgaDbd for a class that can be used to send
    messages of this type to a receiver.
*/
class CUbxPollMgaDbd : public CUbxMsg
{
public: // Functions
  //! Constructor
  CUbxPollMgaDbd(void *context, WRITE_FUNC_p pWrite, size_t maxMsg);

  //! Destructor
  virtual ~CUbxPollMgaDbd();

  //! Extract data from the object retrieved from the receiver
  ssize_t getData(unsigned char **buf);

  //! Does the object contain data retrieved from the receiver
  bool hasData();

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

private:                   // Variables
  CLockedStorage _storage; //!< Data retrieved from the receiver
  bool _hasAllData;        //!< Indicates if all data is in _storage
  size_t _rcvdDbdMsgs;     //!< Counts the messages received
};
#endif //__UBXPOLLMGADBD__
