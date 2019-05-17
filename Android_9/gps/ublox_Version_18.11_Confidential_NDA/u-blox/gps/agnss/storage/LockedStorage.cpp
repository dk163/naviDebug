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
    \ref CLockedStorage implementation

    \brief
    Implementation of \ref CLockedStorage, a class that stores data and makes
    sure no race condition occurs while retrieving, overwriting or appending to
    to the data.
*/

#include "LockedStorage.h"
#include "../helper/helperFunctions.h"
#include <limits.h>
#include <mutex>
#include <stdlib.h>
#include <string.h>

//lint -sem( CLockedStorage::lock, thread_lock )
//lint -sem( CLockedStorage::unlock, thread_unlock )

/*! Default constructor
*/
CLockedStorage::CLockedStorage() : _checksum(0), _data(NULL), _size(0) {}

/*! Destructor
*/
CLockedStorage::~CLockedStorage()
{
  // Make sure nothing happens during clean up
  lock();
  _checksum = 0;
  free(_data);
  _data = NULL;
  _size = 0;
  unlock();
}

/*! Append data to the currently stored data, if available.
    If no data is stored in the object, it will behave
    identically to \ref set.

    \param pData           : The data that should be appended to the
                             the stored data. Must not be NULL
    \param iData           : Number of bytes to append. Must not be 0
    \param checksum        : If passed to the function the checksum of
                             the complete stored data set will be
                             stored in the memory location this pointer
                             is pointing at. May be NULL
*/
bool CLockedStorage::append(unsigned char const *pData,
                            size_t iData,
                            uint16_t *const checksum /*=NULL*/)
{
  if (!pData || !iData)
    return false;

  bool result = false;
  lock(); //LOCK
  unsigned char *oldData = NULL;
  ssize_t oldSize = getCopy_nolock(&oldData, NULL);
  // Is thhere is already data stored in this object?
  if (oldSize >= 0)
  {
    // Yes - append the new data
    unsigned char *tmpData = (unsigned char *)realloc(oldData, oldSize + iData);
    if (tmpData)
    {
      oldData = tmpData;
      //lint -e(449)
      memcpy(oldData + oldSize, pData, iData);
      result = set_nolock(oldData, oldSize + iData, checksum);
    }
    //lint -e(449)
    free(oldData);
  }
  else if (oldSize == LOCKED_STORAGE_NO_DATA)
  {
    // No - set_nolock() will handle this
    result = set_nolock(pData, iData, checksum);
  }
  unlock(); //UNLOCK
  return result;
}

/*! Returns if the system contains data
    Warning: It is possible that this changes between this call
    and the next interactions to the object. This can however
    be a helpful if it is e.g. known that the storage is never
    cleared and it is important to know if it is worth continuing
    indipendently of the actual data stored.
    \return                     true if data is stored
                                false otherwise
*/
bool CLockedStorage::hasData()
{
  lock(); //LOCK
  bool result = (_data != NULL);
  unlock(); //UNLOCK
  return result;
}

/*! Store data in the container. The passed data will be copied
    and will be kept consistent by locks afterwards.

    \param pData           : The data that should be stored.
                             If the argument is NULL and iData is 0
                             this is identical to \ref clear
    \param iData           : Number of bytes to store.
    \param checksum        : If passed to the function the checksum of
                             the complete stored data set will be
                             stored in the memory location this pointer
                             is pointing at. May be NULL
    \return                  true on success, false otherwise
*/
bool CLockedStorage::set(unsigned char const *pData,
                         size_t iData,
                         uint16_t *const checksum /*=NULL*/)
{
  lock(); //LOCK
  bool result = set_nolock(pData, iData, checksum);
  unlock(); //UNLOCK
  return result;
}

/*! Store data in the container without using locks. Used
    by \ref set in combination with \ref lock and \ref unlock

    \param pData           : The data that should be stored.
                             If the argument is NULL and iData is 0
                             this is identical to \ref clear
    \param iData           : Number of bytes to store.
    \param checksum        : If passed to the function the checksum of
                             the complete stored data set will be
                             stored in the memory location this pointer
                             is pointing at. May be NULL
    \return                  true on success, false otherwise
*/
bool CLockedStorage::set_nolock(unsigned char const *pData, size_t iData, uint16_t *const checksum)
{
  if (!((pData && iData) || (!pData && iData == 0)) || iData > SSIZE_MAX)
    return false;

  bool result = false;
  //lint -e(173)
  unsigned char *tmp_data = NULL;
  bool cont_possible = true;
  // Try to get new storage
  // space for the new data, if required
  if (pData)
  {
    tmp_data = (unsigned char *)malloc(iData);
    if (!tmp_data)
      cont_possible = false;
  }

  // This makes sure that tmp_data was allocated successfully
  // if required
  if (cont_possible)
  {
    // Replace the old data with
    // the new (if there is)
    free(_data);
    _size = iData;
    _data = tmp_data;
    tmp_data = NULL;
    // If there is data to be stored,
    // copy it to new storage
    if (pData && _data)
    {
      memcpy(_data, pData, iData);
      // Calculate the ckecsum
      _checksum = calculateChecksum(_data, _size);
    }
    else
    {
      _checksum = 0;
    }
    // If a checksum is requested, fill it
    if (checksum)
    {
      *checksum = _checksum;
    }
    result = true;
  }
  free(tmp_data);
  return result;
}

/*! Get a copy of the stored data from the container.

    \param pData           : The pointer which should point to the
                             copy of the data on success..
                             Must not be NULL
    \param checksum        : If passed to the function the checksum of
                             the complete stored data set will be
                             stored in the memory location this pointer
                             is pointing at. May be NULL
    \return                  On success the number of bytes copied,
                             otherwise either
                             - LOCKED_STORAGE_ERROR
                             - LOCKED_STORAGE_NO_DATA
*/
ssize_t CLockedStorage::getCopy(unsigned char **const pData, uint16_t *const checksum /*=NULL*/)
{
  lock(); //LOCK
  ssize_t result = getCopy_nolock(pData, checksum);
  unlock(); //UNLOCK
  return result;
}

/*! As \ref getCopy but without using locks. \ref getCopy uses
    this together with \ref lock and \ref unlock

    \param pData           : The pointer which should point to the
                             copy of the data on success..
                             Must not be NULL
    \param checksum        : If passed to the function the checksum of
                             the complete stored data set will be
                             stored in the memory location this pointer
                             is pointing at. May be NULL
    \return                  On success the number of bytes copied,
                             otherwise either
                             - LOCKED_STORAGE_ERROR
                             - LOCKED_STORAGE_NO_DATA
*/
ssize_t CLockedStorage::getCopy_nolock(unsigned char **const pData, uint16_t *const checksum)
{
  if (!pData)
  {
    return LOCKED_STORAGE_ERROR;
  }

  ssize_t result = LOCKED_STORAGE_ERROR;
  if (_data)
  {
    // Try to acquire a storage
    // for the copy of the data stored
    unsigned char *tmp_data = (unsigned char *)malloc(_size);
    if (tmp_data)
    {
      memcpy(tmp_data, _data, _size);
      *pData = tmp_data;
      result = _size;
      // Copy checksum
      if (checksum)
      {
        *checksum = _checksum;
      }
    }
  }
  else
  {
    result = LOCKED_STORAGE_NO_DATA;
  }
  return result;
}

/*! Lock the storage
*/
void CLockedStorage::lock() { _mutex.lock(); }

/*! Unlock the storage
*/
void CLockedStorage::unlock() { _mutex.unlock(); }

/*! Clear the content of the storage
    this is identical to a call to
    \ref set with all options set to
    NULL respectively 0
*/
void CLockedStorage::clear() { set(NULL, 0); }
