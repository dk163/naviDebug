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
 * $Id: LockedStorage.h 113752 2016-04-06 12:37:28Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/storage/LockedStorage.h $
 *****************************************************************************/
 
/*! \file
    \ref CLockedStorage definition 

    \brief
    Definition of \ref CLockedStorage, a class that stores data and makes sure
    no race condition occurs while retrieving, overwriting or appending to
    to the data.
*/

#ifndef UBX_STORAGE
#define UBX_STORAGE
#include <pthread.h>
#include <stdint.h>

#define LOCKED_STORAGE_ERROR -1       //!< An error occured during this call
#define LOCKED_STORAGE_NO_DATA -2     //!< No data is stored in this object

class CTimeHandler;
class CPositionHandler;

/*! \class CLockedStorage
    \brief Stores data and avoids race conditions while retrieving,
    overwriting or appending the stored data.
*/
class CLockedStorage
{
    public: // Functions
        //! Default Constructor
        CLockedStorage();

        //! Destructor
        virtual ~CLockedStorage();

        //! Set the data of the storage
        bool set(unsigned char const * pData, size_t iData, uint16_t * const checksum=NULL);

        //! Append data to the storage
        bool append(unsigned char const * pData, size_t iData, uint16_t * const checksum=NULL);

        //! Does the storage contain data?
        bool hasData();

        //! Get a copy of the stored data
        ssize_t getCopy(unsigned char ** const pData, uint16_t * const checksum=NULL);

        //! Clear the contents of the storage
        void clear();

    private: // Friends
        //! Wrapper for this class
        friend class CTimeHandler;

        //! Wrapper for this class
        friend class CPositionHandler;

    private: // Functions
        //! Lock the data in the storage
        void lock();

        //! Unlock the data in the storage
        void unlock();

        //! \ref set withtout locking the storage
        bool set_nolock(unsigned char const * pData, size_t iData, uint16_t * const checksum);

        //! \ref getCopy without locking the storage
        ssize_t getCopy_nolock(unsigned char ** const pData, uint16_t * const checksum);

    private: // Variables
        uint16_t _checksum;          //!< The checksum of the current data
        unsigned char * _data;       //!< The stored data
        size_t _size;                //!< The size of the stored data
        pthread_mutex_t _mutex;      //!< The mutex to lock the data
};

#endif //UBX_STORAGE

