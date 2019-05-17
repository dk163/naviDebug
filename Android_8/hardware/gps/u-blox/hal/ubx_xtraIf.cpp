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
 * $Id: ubx_xtraIf.cpp 94373 2015-04-16 13:28:53Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_xtraIf.cpp $
 *****************************************************************************/

#include "ubx_xtraIf.h"
#include "ubx_moduleIf.h"
#include "ubxgpsstate.h"

///////////////////////////////////////////////////////////////////////////////

static CXtraIf s_myIf;

const GpsXtraInterface CXtraIf::s_interface = {
    IF_ANDROID23( size:		sizeof(GpsXtraInterface), )
	init:					CXtraIf::init,
	inject_xtra_data:		CXtraIf::injectData,
};

CXtraIf::CXtraIf()
{
	m_ready = false;
    memset(&m_callbacks, 0, sizeof(m_callbacks));
}

int CXtraIf::init(GpsXtraCallbacks* callbacks)
{
    if (s_myIf.m_ready)
        UBX_LOG(LCAT_ERROR, "already initialized");
    UBX_LOG(LCAT_VERBOSE, "");
    s_myIf.m_callbacks = *callbacks;
	s_myIf.m_ready = true;
	return 0;
//lint -e{818} remove Pointer parameter 'callbacks' (line 46) could be declared as pointing to const
}

int CXtraIf::injectData(char* data, int length)
{
	UBX_LOG(LCAT_VERBOSE, "length=%d", length);

	CUbxGpsState* pUbxGps = CUbxGpsState::getInstance();
	pUbxGps->lock();
	pUbxGps->putNewOfflineData((const unsigned char*)data, (unsigned int) length);
	pUbxGps->unlock();
	
	return 0;
//lint -e{818} remove Pointer parameter 'data' could be declared as pointing to const
}

void CXtraIf::requestDownload(void)
{
    if (!s_myIf.m_ready)
    {
        UBX_LOG(LCAT_ERROR, "class not initialized");
        return;
    }
    UBX_LOG(LCAT_VERBOSE, "");
    s_myIf.m_callbacks.download_request_cb();
}

