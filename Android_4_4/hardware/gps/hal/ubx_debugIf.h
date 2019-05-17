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
 * $Id: ubx_debugIf.h 83032 2014-07-14 16:14:58Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_debugIf.h $
 *****************************************************************************/

#ifndef __UBX_DEBUGIF_H__
#define __UBX_DEBUGIF_H__

#include "std_inc.h"

#if defined CDEBUGIF_EN

class CDebugIf
{
public:
    static const void *getIf() { return &s_interface; }

private:
	// interface
    static size_t getInternalState(char* buffer, size_t bufferSize);

	// variables
    static const GpsDebugInterface s_interface;
};

#endif

#endif /* __UBX_DEBUGIF_H__ */
