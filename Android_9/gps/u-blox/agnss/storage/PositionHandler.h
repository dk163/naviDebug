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
    \ref CPositionHandler definition

    \brief
    Definition of \ref CPositionHandler, a class that ages the injected reliable positions
    based on \ref CTimeHandler
*/

#ifndef __UBX_POSITIONHANDLER_H__
#define __UBX_POSITIONHANDLER_H__
#include "../helper/helperTypes.h"
#include "LockedStorage.h"
#include "TimeHandler.h"
#include <pthread.h>
#include <time.h>

#define INACC_POS_RATIO 2000LL //!< Estimated position change: ~20m/s

/*! \class CPositionHandler
    \brief ages the injected reliable times based
*/
class CPositionHandler
{
public:
  //! Constructor of this wrapper function
  CPositionHandler(CLockedStorage &storage, CTimeHandler &timeH);

  //! Destructor
  virtual ~CPositionHandler();

  //! Indicates if a valid position is available
  bool hasValidPosition();

  //! Inject the current position into this object for aging
  bool injectPosition(POS_t const *pos);

  //! Retrieve the originally injected position
  bool getInjectedPosition(POS_t *pos, ACCTIME_t *stampDiff = NULL);

  //! Get the current position as calculated from the injected one
  bool getCurrentPosition(POS_t *pos);

private:
  //! Extract the STAMPED_TIME_t from storage
  static bool parse(CLockedStorage &storage, STAMPED_POS_t &pos);

  //! Do the actual work for \ref parse, but without locking storage
  static bool parse_nolock(CLockedStorage &storage, STAMPED_POS_t &pos);

  //! Is the provided stamped position valid?
  static bool isValid(STAMPED_POS_t const *stPos);

  CLockedStorage &_storage; //!< The storage this object acts as a wrapper for
  CTimeHandler &_time;      //!< The time source of this object
};
#endif // __UBX_POSITIONHANDLER_H__
