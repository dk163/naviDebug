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
 * $Id: ubx_debugIf.cpp 83032 2014-07-14 16:14:58Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_debugIf.cpp $
 *****************************************************************************/

#include "ubx_debugIf.h"

#if defined CDEBUGIF_EN

static CDebugIf s_myIf;

const GpsDebugInterface CDebugIf::s_interface = {
    size:                   sizeof(GpsDebugInterface),
    get_internal_state:     getInternalState,
};

size_t CDebugIf::getInternalState(char* buffer, size_t bufferSize)
{
    ((void) (buffer));
    UBX_LOG(LCAT_VERBOSE, "size=%zd", bufferSize);
    return 0;
}

#endif
