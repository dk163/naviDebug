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
 * $Id: ubx_niIf.h 103412 2015-10-01 08:57:15Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_niIf.h $
 *****************************************************************************/

#ifndef __UBX_NIIF_H__
#define __UBX_NIIF_H__

#include "std_inc.h"
#include <semaphore.h>


class CNiIf
{
public:
	CNiIf();
    static const void* getIf(void) { return &s_interface; }
	static CNiIf* getInstance(void);
	
	// callbacks
	static void request(GpsNiNotification* pNotification);
    sem_t sem;
    int m_cmd;

private:
	// interface
    static void init(GpsNiCallbacks* callbacks);
	static void respond(int notif_id, GpsUserResponseType user_response);

	// variables
    static const GpsNiInterface s_interface;
	GpsNiCallbacks m_callbacks;
	bool m_ready;
    pthread_t m_thread;
	
	//impelementation
	static void timoutThread(void *pThreadData);
};

#endif /* __UBX_DEBUGIF_H__ */
