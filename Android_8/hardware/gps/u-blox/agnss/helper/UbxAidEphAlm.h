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
 * $Id: UbxAidEphAlm.h 103715 2015-10-06 11:18:18Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/helper/UbxAidEphAlm.h $
 *****************************************************************************/
 
/*! \file
    \ref CUbxAidEphAlm definition 

    \brief
    Definition of \ref CUbxAidEphAlm, a class to send Ephemeris and Alamanac
    data to receivers supporting the UBX-AID-* commands
*/
#ifndef __UBXAIDEPHALM__
#define __UBXAIDEPHALM__
#include "UbxMsg.h"
#include "helperFunctions.h"
#include "../storage/LockedStorage.h"

/*! \class CUbxAidEphAlm

    \brief
    Definition of \ref CUbxAidEphAlm, a class derived from \ref CUbxMsg,
    which is used to send Ephemeris and Alamanac to a receiver supporting
    UBX-AID-* messages. Please refer to \ref CUbxPollAidEphAlm for a class
    that can be used to poll messages of this type.
*/
class CUbxAidEphAlm : public CUbxMsg
{
    public: // Functions
        //! Constructor
        CUbxAidEphAlm(void * context, WRITE_FUNC_p pWrite, size_t maxMsg, bool autoClear = true);

        //! Destructor
        //lint -sem(CUbxAidEphAlm::impl_clearData,cleanup)    
        virtual ~CUbxAidEphAlm();

        //! Set the data to be transferred
        bool setData(unsigned char const *buf, size_t size);

    private: // Functions
        /////////////////////////////////////////////////////////////////
        // Start function declarations as required to implemet by base //
        /////////////////////////////////////////////////////////////////
        bool impl_writeUbxMsg();
        TRI_STATE_FUNC_RESULT_t impl_isExpectedAnswer(unsigned char const * buf, size_t size);
        void impl_clearData();
        ////////////////////////////////////////////////////////////////
        // Stop function declarations as required to implemet by base //
        ////////////////////////////////////////////////////////////////

    private: // Variables
        unsigned char * _buf;        //!< Buffer containing the messages to be sent
        BUFC_t * _msgs;              //!< Array of pointers to each message in _buf
        size_t _sentMsgs;            //!< Counts the messages already sent in _msgs
        size_t _sendableMsgs;        //!< Number of messages in _msgs

        static const struct
               UBX_MSG_TYPE _allowed[]; //!< Allowed message types in _buf
};
#endif //__UBXAIDEPHALM__
