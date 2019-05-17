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
    \ref CTimeHandler definition

    \brief
    Definition of \ref CTimeHandler, a class that ages the injected reliable
    times based on a monotonic system clock and acts as a wrapper to
    \ref CLockedStorage
*/

#ifndef __UBX_TIMEHANDLER_H__
#define __UBX_TIMEHANDLER_H__
#include "../helper/helperTypes.h"
#include "LockedStorage.h"
#include <pthread.h>
#include <time.h>

#define INACC_TIME_RATIO 10000000LL //!< Estimated clock drift rate 10 ppm

/*! \class CTimeHandler
    \brief ages the injected reliable times based
*/
class CTimeHandler
{
public: // Functions
  //! Constructor
  CTimeHandler(CLockedStorage &storage);

  //! Destructor
  virtual ~CTimeHandler();

  //! Is there a valid time stored?
  bool hasValidTime();

  //! Inject a new current time
  bool injectTime(ACCTIME_t const *accTime);

  //! Get the originally injected time
  bool getInjectedTime(ACCTIME_t *accTime, struct timespec *stampDiff = NULL);

  //! Get the current time calculated from the injected time
  bool getCurrentTime(ACCTIME_t *accTime);

private: // Functions
  //! Extract the \ref STAMPED_TIME_t from storage
  static bool parse(CLockedStorage &storage, STAMPED_TIME_t &stTime);

  //! Extract \ref STAMPED_TIME_t from storage without locking
  static bool parse_nolock(CLockedStorage &storage, STAMPED_TIME_t &stTime);

  //! Is the passed \ref STAMPED_TIME_t valid?
  static bool isValid(STAMPED_TIME_t const *stTime);

private:                    // Variables
  CLockedStorage &_storage; //!< The storage this objects as a wrapper for
};
#endif // __UBX_TIMEHANDLER_H__
