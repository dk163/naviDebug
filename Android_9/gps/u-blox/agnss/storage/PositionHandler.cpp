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
    \ref CPositionHandler implementation

    \brief
    Implementation of \ref CPositionHandler, a class that ages the injected reliable positions
    based on \ref CTimeHandler
*/

#include "PositionHandler.h"
#include "../helper/helperFunctions.h"
#include <stdlib.h>
#include <string.h>

/*! Creates an instance of this storage-wrapper-class
    \param storage         : Reference to the storage from where timing
                             information will be read and to which it will
                             be written, depending which functions of this
                             objects are called.
    \param timeH           : The \ref CTimeHandler from which the current
                             time will be requested if timestamping of
                             a position is required.
*/
CPositionHandler::CPositionHandler(CLockedStorage &storage, CTimeHandler &timeH)
  : _storage(storage), _time(timeH)
{
}

/*! Tears down this object
*/
CPositionHandler::~CPositionHandler() {}

/*! Indicates with its return value if a valid position is available
    \return                  true if successful, false otherwise
*/
bool CPositionHandler::hasValidPosition()
{
  POS_t pos;
  return getCurrentPosition(&pos);
}

/*! When passed a valid position, this is assumed to be the current
    position which will be aged with \ref INACC_POS_RATIO cm/s
    \param pos             : The current position. Must not be NULL
    \return                  true on success, false otherwise
*/
bool CPositionHandler::injectPosition(POS_t const *pos)
{
  if (!isValidPos(pos))
    return false;

  _storage.lock(); // LOCK
  bool result = false;
  STAMPED_POS_t newStPos;
  memset(&newStPos, 0, sizeof(newStPos));

  // Use the current time as timestamp
  if (_time.getCurrentTime(&newStPos.storeTime))
  {
    memcpy(&newStPos.pos, pos, sizeof(newStPos.pos));
    result = _storage.set_nolock((unsigned char const *)&newStPos, sizeof(newStPos), NULL);
  }

  _storage.unlock(); // UNLOCK

  return result;
}

/*! Get the exact position value that was passed to this object with
    \ref injectPosition at an earlier point in time.
    \param pos             : A pointer to the variable which should be filled
                             with the originally injected position. Must not
                             be NULL.
    \param timeDiff        : The time that has passed since the injection of
                             the position. The accuracy values of the
                             injection time and the current time are added
    \return                  true on success, false otherwise
*/
bool CPositionHandler::getInjectedPosition(POS_t *pos, ACCTIME_t *timeDiff /* = NULL */)
{
  if (!pos)
    return false;

  ACCTIME_t now;
  STAMPED_POS_t tmpPos;
  bool result = false;
  // Position information is only valid if the current time
  // is known as well as if the earliest possible storetime
  // is earlier than the current time
  if (_time.getCurrentTime(&now) && parse(_storage, tmpPos))
  {
    // Is the current time later than the store time?
    if (timecmp(&now.time, &tmpPos.storeTime.time) >= 0)
    {
      ACCTIME_t tmpTimeDiff;
      // Difference between the store time and now as well as the combined
      // inaccuracy
      tmpTimeDiff.time.tv_sec = now.time.tv_sec - tmpPos.storeTime.time.tv_sec;
      tmpTimeDiff.time.tv_nsec = now.time.tv_nsec - tmpPos.storeTime.time.tv_nsec;
      tmpTimeDiff.acc.tv_sec = now.acc.tv_sec + tmpPos.storeTime.acc.tv_sec;
      tmpTimeDiff.acc.tv_nsec = now.acc.tv_nsec + tmpPos.storeTime.acc.tv_nsec;
      normalizeTime(&tmpTimeDiff.time);
      normalizeTime(&tmpTimeDiff.acc);

      if (timeDiff)
      {
        memcpy(timeDiff, &tmpTimeDiff, sizeof(*timeDiff));
      }
      memcpy(pos, &tmpPos.pos, sizeof(*pos));
      result = true;
    }
  }

  return result;
}

/*! Get the current position. The current position is the injected position
    with its accuracy adjusted by a value proportional to the passed time.
    This adjustment factor is \ref INACC_POS_RATIO cm/s.
    \param pos             : A pointer to the variable into which the current
                             position estimation must be written.
                             Must not be NULL
    \return                  true on success, false otherwise.
*/
bool CPositionHandler::getCurrentPosition(POS_t *pos)
{
  if (!pos)
    return false;

  bool result = false;
  POS_t injPos;
  ACCTIME_t stampDiff;
  if (getInjectedPosition(&injPos, &stampDiff))
  {

    // The difference between the earliest possible store time and
    // the latest possible current time to calculate the maximum
    // position inaccuracy
    stampDiff.time.tv_sec += stampDiff.acc.tv_sec;
    stampDiff.time.tv_nsec += stampDiff.acc.tv_nsec;
    normalizeTime(&stampDiff.time);

    // Position stays the same and the accuracy is required as well
    POS_t tmpPos;
    memcpy(&tmpPos, &injPos, sizeof(tmpPos));
    // Increase the inaccuracy of the provided location by the difference between
    // the two times and thir inaccuracy
    tmpPos.posAccCm += INACC_POS_RATIO * stampDiff.time.tv_sec +
                       (INACC_POS_RATIO * stampDiff.time.tv_nsec) / NSEC_PER_SEC;

    // Make sure the accuracy fits into this variable and is within the valid range
    if (tmpPos.posAccCm >= injPos.posAccCm && tmpPos.posAccCm)
    {
      memcpy(pos, &tmpPos, sizeof(*pos));
      result = true;
    }
  }

  return result;
}

/*! Parse the data stored in storage and provide it to the caller
    given the stored position is valid. To be valid, the data in storage must:
    - be exactly the size of a \ref STAMPED_POS_t struct
    - have a set valid flag
    - the time timestamp value must be smaller than the current timestamp
    \param storage    : Reference to the storage that should be parsed
    \param stPos      : Reference to the time to which the parsed data
                        should be written on success
    \return             true on success, false otherwise
*/
bool CPositionHandler::parse(CLockedStorage &storage, STAMPED_POS_t &stPos)
{
  storage.lock();
  bool result = parse_nolock(storage, stPos);
  storage.unlock();
  return result;
}

/*! Version of \ref parse which is not locking storage before accessing
    its data. Be very careful when using this function.
*/
bool CPositionHandler::parse_nolock(CLockedStorage &storage, STAMPED_POS_t &stPos)
{
  bool result = false;
  unsigned char *data = NULL;
  ssize_t dataSize = storage.getCopy_nolock(&data, NULL);
  // If the extracted data has not the correct size
  // don't do anything else
  STAMPED_POS_t storedPos;
  if (dataSize == sizeof(storedPos))
  {
    memcpy(&storedPos, data, sizeof(storedPos));
    // Make sure the stored position is valid in every respect before returning
    if (isValid(&storedPos))
    {
      memcpy(&stPos, &storedPos, sizeof(stPos));
      result = true;
    }
  }
  free(data);
  return result;
}

/*! Checks if the provided stamped position is valid
    \param stPos           : The position that has to be checked for
                             validity. Must not be NULL.
    \return                  true if valid, false otherwise
*/
bool CPositionHandler::isValid(STAMPED_POS_t const *stPos)
{
  bool result = false;
  if (stPos && isValidPos(&stPos->pos) && isValidAccTime(&stPos->storeTime))
    result = true;
  return result;
}
