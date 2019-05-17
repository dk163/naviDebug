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
 * $Id: TimeHandler.cpp 103715 2015-10-06 11:18:18Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/storage/TimeHandler.cpp $
 *****************************************************************************/
 
/*! \file
    \ref CTimeHandler implementation

    \brief
    Implementation of \ref CTimeHandler, a class that ages the injected
    reliable times based on a monotonic system clock and acts as a wrapper to
    \ref CLockedStorage
*/

#include <string.h>
#include <stdlib.h>
#include "TimeHandler.h"
#include "../helper/helperFunctions.h"

/*! Contstructor
    \param storage         : Storage for which this objets acts as wrapper for
*/
CTimeHandler::CTimeHandler(CLockedStorage &storage)
    : _storage(storage)
{
}

/*! Destructor
*/
CTimeHandler::~CTimeHandler()
{
}

/*! Informs the caller if there is a valid time stored in this object
    \return                  true if there is a valid time, false otherwise
*/
bool CTimeHandler::hasValidTime()
{
    STAMPED_TIME_t tmpTime;
    return parse(_storage, tmpTime);
}

/*! Inject the current time to the object
    \param accTime         : The current time. The argument must be valid
                             and not be NULL.
    \return                  true on success, false otherwise
*/
bool CTimeHandler::injectTime(ACCTIME_t const * accTime)
{
    if(!isValidAccTime(accTime))
        return false;

    _storage.lock(); // LOCK
    bool result=false;

    // Is there a valid stored value? If yes make sure the injected
    // time has a newer timestamp. Otherwise don't accept that time
    STAMPED_TIME_t oldStTime;
    bool validOldTime=parse_nolock(_storage, oldStTime);

    STAMPED_TIME_t newStTime;
    if( getMonotonicCounter(&newStTime.stamp)
     && (!validOldTime || (validOldTime && timecmp(&oldStTime.stamp, &newStTime.stamp)<=0)) )
    {
        memcpy(&newStTime.accTime, accTime, sizeof(newStTime.accTime));
        normalizeTime(&newStTime.accTime.time);
        normalizeTime(&newStTime.accTime.acc);
        result=_storage.set_nolock((unsigned char const *) &newStTime, sizeof(newStTime), NULL);
    }    

    _storage.unlock(); // UNLOCK

    return result;
}

/*! Get the exact time value that was passed to this object with
    \ref injectTime at an earlier point in time.
    \param accTime         : A pointer to the variable which should be filled
                             with the originally injected time. Must not
                             be NULL.
    \param stampDiff       : The time that has passed since the injection of
                             the time. The accuracy values of the
                             injection time and the current time are added
    \return                  true on success, false otherwise
*/
bool CTimeHandler::getInjectedTime(ACCTIME_t * accTime, struct timespec * stampDiff /* = NULL */)
{
    if(!accTime)
        return false;

    bool result=false;
    STAMPED_TIME_t stTime;
    if(parse(_storage, stTime))
    {
        // does the time difference from the storage time have to be calculated?
        bool succ=true;
        if(stampDiff)
        {
            struct timespec nowStamp;
            if( !getMonotonicCounter(&nowStamp) )
            {
                succ=false;
            }
            else
            {
                // The current monotonic time must always be bigger
                // than the stored one.
                stampDiff->tv_sec=nowStamp.tv_sec-stTime.stamp.tv_sec;
                // This might become negative, but thats no problem
                stampDiff->tv_nsec=nowStamp.tv_nsec-stTime.stamp.tv_nsec;
                // No negative or too big nanosecond values
                normalizeTime(stampDiff);
            }
        }

        // If stampDiff didn't fail or wasn't calculated, make sure
        // the time with accuracy information is returned to the
        // user
        if(succ)
        {
            memcpy(accTime, &stTime.accTime, sizeof(*accTime));
            result=true;
        }
    }
    return result;
}

/*! Get the current time. The current time is the injected time
    with its accuracy adjusted by a value proportional to the passed time as
    well as the actual time value adjusted by the seconds and nanoseconds
    passed.
    This adjustment factor is \ref INACC_TIME_RATIO^-1 ppm
    \param accTime         : A pointer to the variable into which the current
                             time estimation must be written.
                             Must not be NULL
    \return                  true on success, false otherwise.
*/
bool CTimeHandler::getCurrentTime(ACCTIME_t * accTime)
{
    if(!accTime)
        return false;

    bool result=false;
    ACCTIME_t injAccTime;
    struct timespec stampDiff;
    if(getInjectedTime(&injAccTime, &stampDiff))
    {
        // Add the difference in the timestamps to the injection time
        // to get the estimation for the current time
        accTime->time.tv_sec = injAccTime.time.tv_sec + stampDiff.tv_sec;
        accTime->time.tv_nsec = injAccTime.time.tv_nsec + stampDiff.tv_nsec;
        accTime->leapSeconds = injAccTime.leapSeconds;
        accTime->valid = true;

        // Increase the inaccuracy of the provided time by the
        // estimated clock drift rate multiplied with the time
        // difference to the last inject
        accTime->acc.tv_sec = injAccTime.acc.tv_sec
                            + stampDiff.tv_sec / INACC_TIME_RATIO;
        accTime->acc.tv_nsec = injAccTime.acc.tv_nsec
                             + (NSEC_PER_SEC / INACC_TIME_RATIO) * stampDiff.tv_sec
                             + stampDiff.tv_nsec / INACC_TIME_RATIO;

        // Normalize the calculated times
        normalizeTime(&accTime->time);
        normalizeTime(&accTime->acc);

        result=true;
    }

    return result;
}

/*! Parse the data stored in storage and provide it to the caller
    given the stored time is valid. To be valid, the data in storage must:
    - be exactly the size of a \ref STAMPED_TIME_t struct
    - have a set valid flag
    - the time timestamp value must be smaller than the current timestamp
    - the actual time must be later than the earliest possible time (2015.01.01)
    \param storage    : Reference to the storage that should be parsed
    \param stTime     : Reference to the time to which the parsed data should be
                        should be written on success
    \return             true on success, false otherwise
*/
bool CTimeHandler::parse(CLockedStorage &storage, STAMPED_TIME_t &stTime)
{
    storage.lock();
    bool result=parse_nolock(storage, stTime);
    storage.unlock();
    return result;
}

/*! Version of \ref parse which is not locking storage before accessing
    its data. Be very careful when using this function.
    \param storage    : Reference to the storage that should be parsed
    \param stTime     : Reference to the time to which the parsed data
                        should be written on success
    \return             true on success, false otherwise
*/
bool CTimeHandler::parse_nolock(CLockedStorage &storage, STAMPED_TIME_t &stTime)
{
    bool result=false;
    unsigned char *data=NULL;
    ssize_t dataSize=storage.getCopy_nolock(&data, NULL);
    // If the extracted data has not the correct size
    // don't do anything else
    STAMPED_TIME_t storedTime;
    if(dataSize==sizeof(storedTime))
    {
        memcpy(&storedTime, data, sizeof(storedTime));
        // Make sure the stored time is valid in every respect before returning
        if(isValid(&storedTime))
        {
            memcpy(&stTime, &storedTime, sizeof(stTime));
            result=true;
        }
    }
    free(data);
    return result;
}

/*! Checks if the provided stamped time is valid
    \param stTime          : The time that has to be checked for
                             validity. Must not be NULL.
    \return                  true if valid, false otherwise
*/
bool CTimeHandler::isValid(STAMPED_TIME_t const * stTime)
{
    if(!stTime || !isValidAccTime(&stTime->accTime))
        return false;

    bool result=false;
    struct timespec now;
    if( getMonotonicCounter(&now)
     && timecmp(&now, &stTime->stamp) > 0) // Mon. now later than stored?
        result=true;

    return result;
}

