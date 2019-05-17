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
 * $Id: UbxMgaIniPosLlh.h 103715 2015-10-06 11:18:18Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/agnss/helper/UbxMgaIniPosLlh.h $
 *****************************************************************************/
 
/*! \file
    \ref CUbxMgaIniPosLlh definition 

    \brief
    Definition of \ref CUbxMgaIniPosLlh, a class to send position
    nformation to the receiver, if it supports MGA-INI-* commands
*/
#ifndef __UBXMGAINIPOSLLH__
#define __UBXMGAINIPOSLLH__
#include <time.h>
#include "UbxMsg.h"
#include "helperFunctions.h"

/*! \class CUbxMgaIniPosLlh

    \brief
    Definition of \ref CUbxMgaIniPosLlh, a class derived from \ref CUbxMsg,
    which is used to send position information to a receiver
    supporting UBX-MGA-INI* messages.
*/
class CUbxMgaIniPosLlh : public CUbxMsg
{
    public: // Functions
        //! Constructor
        CUbxMgaIniPosLlh(void * context, WRITE_FUNC_p pWrite, size_t maxMsg, bool autoClear = true);

        //! Destructor
        virtual ~CUbxMgaIniPosLlh();

        //! Set the current position
        bool setPos(POS_t const * pos);

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

        //! Send position aiding data to the receiver
        bool writeUbxMgaIniPos(POS_t const * pos);

    private: // Variables
        POS_t _pos;              //!< The position to be sent
};
#endif //__UBXMGAINIPOSLLH__

