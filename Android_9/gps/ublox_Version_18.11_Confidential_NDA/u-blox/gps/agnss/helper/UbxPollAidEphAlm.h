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
    \ref CUbxPollAidEphAlm definition

    \brief
    Definition of \ref CUbxPollAidEphAlm, a class to retrieve Ephemeris and
    Alamanac data from receivers supporting the UBX-AID-* commands
*/
#ifndef __UBXPOLLAIDEPHALM__
#define __UBXPOLLAIDEPHALM__
#include "../storage/LockedStorage.h"
#include "UbxMsg.h"
#include "helperFunctions.h"

/*! \class CUbxPollAidEphAlm

    \brief
    Definition of \ref CUbxPollAidEphAlm, a class derived from \ref CUbxMsg,
    which is used to retrieve Ephemeris and Alamanac from a receiver supporting
    UBX-AID-* messages. Please refer to \ref CUbxAidEphAlm for a class
    that can be used to send messages of this type to a receiver.
*/
class CUbxPollAidEphAlm : public CUbxMsg
{
private: // Definitions
  //! State to define which kind of message is currently expected
  typedef enum {
    STATE_EPH = 0,  //! Ephmeris is expected
    STATE_ALM = 1,  //! Almanac is expected
    _STATE_END_ = 2 //! Number of valid states
  } STATE_t;

public: // Functions
  //! Constructor
  CUbxPollAidEphAlm(void *context, WRITE_FUNC_p pWrite, size_t maxMsg);

  //! Destructor
  virtual ~CUbxPollAidEphAlm();

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
  size_t _rcvdAnswMsgs;    //!< Counter against _expNumMsgs
  STATE_t _state;          //!< Indicates the current state of the object

  static unsigned char const _aidPoll[_STATE_END_][8]; //!< Messages used to poll the receiver
  static size_t const _expNumMsgs;                     //!< Expected number of messages per
                                                       //!< item in \ref STATE_t
};
#endif //__UBXPOLLAIDEPHALM__
