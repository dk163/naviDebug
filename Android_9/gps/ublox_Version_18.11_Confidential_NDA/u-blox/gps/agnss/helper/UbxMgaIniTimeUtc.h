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
    \ref CUbxMgaIniTimeUtc definition

    \brief
    Definition of \ref CUbxMgaIniTimeUtc, a class to send time
    information to the receiver, if it supports UBX-MGA-INI* commands
*/
#ifndef __UBXMGAINITIMEUTC__
#define __UBXMGAINITIMEUTC__
#include "UbxMsg.h"
#include "helperFunctions.h"
#include <time.h>

/*! \class CUbxMgaIniTimeUtc

    \brief
    Definition of \ref CUbxMgaIniTimeUtc, a class derived from \ref CUbxMsg,
    which is used to send time information to a receiver supporting
    UBX-MGA-INI-* messages.
*/
class CUbxMgaIniTimeUtc : public CUbxMsg
{
public: // Functions
  //! Constructor
  CUbxMgaIniTimeUtc(void *context, WRITE_FUNC_p pWrite, size_t maxMsg, bool autoClear = true);

  //! Destructor
  virtual ~CUbxMgaIniTimeUtc();

  //! Set the time to be transferred
  bool setTime(ACCTIME_t const *accTime);

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

  //! Send position aiding information to the receiver
  bool writeUbxMgaIniTimeUtc(ACCTIME_t const *accTime);

private:              // Variables
  ACCTIME_t _accTime; //!< The time that should be sent to the receiver
};
#endif //__UBXMGAINITIMEUTC__
