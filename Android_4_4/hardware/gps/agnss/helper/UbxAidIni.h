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
 * $Id: UbxAidIni.h 103715 2015-10-06 11:18:18Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/helper/UbxAidIni.h $
 *****************************************************************************/
 
/*! \file
    \ref CUbxAidIni definition 

    \brief
    Definition of \ref CUbxAidIni, a class to send time and/or position
    information to the receiver, if it supports UBX-AID-* commands
*/
#ifndef __UBXAIDINI__
#define __UBXAIDINI__
#include <time.h>
#include "UbxMsg.h"
#include "helperFunctions.h"

/*! \class CUbxAidIni

    \brief
    Definition of \ref CUbxAidIni, a class derived from \ref CUbxMsg,
    which is used to send time and position information to a receiver
    supporting UBX-AID-* messages.
*/
class CUbxAidIni : public CUbxMsg
{
    public: // Functions
        //! Constructor
        CUbxAidIni(void * context, WRITE_FUNC_p pWrite, size_t maxMsg, bool autoClear = true);

        //! Destructor
        virtual ~CUbxAidIni();

        //! Set the position and data to be transferred
        bool setPosTime(ACCTIME_t const * accTime, POS_t const * pos = NULL);

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

        //! Do send position and/or time aiding to the receiver
        bool writeUbxAidIni(ACCTIME_t const * accTime, POS_t const * pos);

    private: // Variables
        ACCTIME_t _accTime;      //!< The time information to be sent
        POS_t _pos;              //!< The position information to be sent
};
#endif //__UBXAIDINI__

