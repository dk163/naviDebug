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
 * $Id: ubx_moduleIf.cpp 106222 2015-11-17 14:04:43Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_moduleIf.cpp $
 *****************************************************************************/

/*!
  \file
  \brief Defines the interface to the Android Framework

  Defines the interface of the HAL module to the Android Framework and
  provides the functions to start it up, shut it down, inject
  information and others.
*/

#include <errno.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <new>

#ifndef ANDROID_BUILD
// Needed for Linux build
#include <malloc.h>
#endif

#include "std_types.h"
#include "ubx_log.h"

//#include "ubx_debugIf.h"
#ifdef SUPL_ENABLED
 #include "ubx_niIf.h"
 #include "ubx_agpsIf.h"
#endif
#include "ubx_rilIf.h"
#include "ubx_xtraIf.h"
#include "ubx_moduleIf.h"

#include "gps_thread.h"

#include "version.h"
// For GetVersion() of libParser
#include "parserbuffer.h"
// For mgaGetVersion() of libMGA
#include "../agnss/mga/libMga.h"

static ControlThreadInfo  s_controlThreadInfo = {
	gpsState:               GPS_UNKNOWN,
	cmdPipes:               { -1, -1 },
	cmdResult:              0,
	mainReady:              false,
	threadMainReadyCond:    PTHREAD_COND_INITIALIZER,
	threadMainReadyMutex:   PTHREAD_MUTEX_INITIALIZER,
	threadCmdCompleteCond:  PTHREAD_COND_INITIALIZER,
	threadCmdCompleteMutex: PTHREAD_MUTEX_INITIALIZER,
	clientCount:            0,
	threadDataAccessMutex:  PTHREAD_MUTEX_INITIALIZER
};
static pthread_t s_mainControlThread = (pthread_t)NULL;

#if (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
/*******************************************************************************
 * HAL MODULE
 ******************************************************************************/

/*!
 *  \brief HAL module information for the framework
 *
 *  Every HAL module must define a struct of this type to be loadable
 *  by hw_get_module(). The struct is defined in
 *  $AOSP_HOME/hardware/libhardware/include/hardware/hardware.h
 */
hw_module_t HAL_MODULE_INFO_SYM = {
	tag:					HARDWARE_MODULE_TAG,        // uint32_t
    version_major:			2,                          // uint16_t
    version_minor:			0,                          // uint16_t
    id:						GPS_HARDWARE_MODULE_ID,     // const char *
    name:					"u-blox GPS/GNSS library",  // const char *
    author:					"u-blox AG - Switzerland",  // const char *
    methods:				&CGpsIf::s_hwModuleMethods, // struct hw_module_methods_t *
    dso:					NULL,                       // module's dso
    reserved:				{0}                         // uint32_t *, padding
};

struct hw_module_methods_t CGpsIf::s_hwModuleMethods =
{
    open: CGpsIf::hwModuleOpen // open a specific device
};

int CGpsIf::hwModuleOpen(const struct hw_module_t* module,
						 char const* name,
						 struct hw_device_t** device)
{
    ((void) (name));
    struct gps_device_t *dev = new(std::nothrow) gps_device_t;
    memset(dev, 0, sizeof(*dev));
    dev->common.tag			= HARDWARE_DEVICE_TAG;
    dev->common.version		= 0;
    dev->common.module		= const_cast<struct hw_module_t*>(module);
    dev->common.close		= CGpsIf::hwModuleClose;
    dev->get_gps_interface	= CGpsIf::getIf;
    *device = (struct hw_device_t*) (void *) dev;

    return 0;
}

int CGpsIf::hwModuleClose(struct hw_device_t* device)
{
    delete device;
    return 0;
}
#else // (PLATFORM_SDK_VERSION <= 8 /* <=2.2 */)
const GpsInterface* gps_get_interface()
{
	return CGpsIf::getIf(NULL);
}
#endif

/*******************************************************************************
 * INTERFACE
 ******************************************************************************/

static CGpsIf s_myIf;

const GpsInterface CGpsIf::s_interface = {
    IF_ANDROID23( size:				sizeof(GpsInterface), )
    init:                   		CGpsIf::init,
    start:                  		CGpsIf::start,
    stop:                   		CGpsIf::stop,
#if (PLATFORM_SDK_VERSION <= 8 /* <=2.2 */)
	set_fix_frequency:				CGpsIf::setFixFrequency,
#endif
    cleanup:                		CGpsIf::cleanup,
    inject_time:            		CGpsIf::injectTime,
    IF_ANDROID23( inject_location:	CGpsIf::injectLocation, )
    delete_aiding_data:				CGpsIf::deleteAidingData,
    set_position_mode:				CGpsIf::setPositionMode,
    get_extension:					CGpsIf::getExtension,
};

CGpsIf::CGpsIf()
{
	m_ready = false;
	m_mode = GPS_POSITION_MODE_MS_BASED;
	m_lastStatusValue = GPS_STATUS_NONE;
    m_capabilities = 0;
    memset(&m_callbacks,0,sizeof(m_callbacks));
}

const GpsInterface *CGpsIf::getIf(struct gps_device_t* /*dev*/)
{
   return &s_interface;
}

CGpsIf* CGpsIf::getInstance()
{
	return &s_myIf;
}

GpsPositionMode CGpsIf::getMode(void) const
{
	return m_mode;
};

uint32_t CGpsIf::getCapabilities(void) const
{
	return m_capabilities;
};

#if (PLATFORM_SDK_VERSION <= 8 /* <=2.2 */)
extern "C" void* CGpsIfThread22(void *info)
{
    ubx_thread(info);
    pthread_exit(NULL);
    return NULL;
}
#endif

static const char *GetVersion(void)
{
    return ANDROID_GNSS_DRIVER_VERSION;
}

int CGpsIf::init(GpsCallbacks* callbacks)
{
    if (s_myIf.m_ready)
	{
        UBX_LOG(LCAT_ERROR, "already initialized");
		return 0;	// Report success since we are already initialised
	}

    UBX_LOG(LCAT_VERBOSE, "Driver version %s", GetVersion());
    UBX_LOG(LCAT_VERBOSE, "libMGA version %s", mgaGetVersion());
    UBX_LOG(LCAT_VERBOSE, "libParser version %s", CParserBuffer::GetVersion());

#if (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
	UBX_LOG(LCAT_VERBOSE, "");
	memcpy(&s_myIf.m_callbacks, callbacks,
				(callbacks->size < sizeof(GpsCallbacks)) ? callbacks->size : sizeof(GpsCallbacks));
	if (callbacks->size != sizeof(GpsCallbacks))
		UBX_LOG(LCAT_WARNING, "callback size %zd != %zd", callbacks->size, sizeof(GpsCallbacks));
#endif

	UBX_LOG(LCAT_DEBUG, "(%u): Initializing - pid %i", (unsigned int) pthread_self(), getpid());

#if (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
 	s_myIf.m_capabilities = GPS_CAPABILITY_SCHEDULING;
 #if (PLATFORM_SDK_VERSION >= 14 /* >=4.0 */)
	s_myIf.m_capabilities |= GPS_CAPABILITY_ON_DEMAND_TIME;
 #endif
 #ifdef SUPL_ENABLED
	s_myIf.m_capabilities |=  GPS_CAPABILITY_MSB | GPS_CAPABILITY_MSA;
 #endif
	UBX_LOG(LCAT_VERBOSE, "set_capabilities=%d(%s)", s_myIf.m_capabilities, _LOOKUPSTRX(s_myIf.m_capabilities, GpsCapabilityFlags));
	s_myIf.m_callbacks.set_capabilities_cb(s_myIf.m_capabilities);
    s_mainControlThread = s_myIf.m_callbacks.create_thread_cb("gps thread", ubx_thread, &s_controlThreadInfo);
#else // (PLATFORM_SDK_VERSION <= 8 /* <=2.2 */)
     /* do somthing here */
    s_mainControlThread = (pthread_t)NULL;
    pthread_create(&s_mainControlThread, NULL, CGpsIfThread22, &s_controlThreadInfo);
#endif
	// Wait for ubx-thread to signal that the setup has been finished
	// and it is ready to run
	UBX_LOG(LCAT_DEBUG, "Wait for ubx-thread to get ready");

	pthread_mutex_lock(&s_controlThreadInfo.threadCmdCompleteMutex);
	while(!s_controlThreadInfo.cmdResult)
		pthread_cond_wait(&s_controlThreadInfo.threadCmdCompleteCond,
						  &s_controlThreadInfo.threadCmdCompleteMutex);
    s_myIf.m_ready = (s_controlThreadInfo.cmdResult == 1);

	pthread_mutex_unlock(&s_controlThreadInfo.threadCmdCompleteMutex);
	UBX_LOG(LCAT_DEBUG, "Initialized complete: result %i. ubx-thread %sready", s_myIf.m_ready, s_myIf.m_ready?"":"NOT ");
    if (!s_myIf.m_ready)
    {
        // Init failed -  release resources
		UBX_LOG(LCAT_ERROR, "Initialisation failed! Freeing the ressources");
		controlThreadInfoRelease(&s_controlThreadInfo);
    }
	else // If everything is fine, tell ubx_thread to continue
	{
		// Tell the ubx-thread that the main-thread is ready
		UBX_LOG(LCAT_DEBUG, "Signal to the waiting ubx-thread that main-thread is ready");
		pthread_mutex_lock(&s_controlThreadInfo.threadMainReadyMutex);
		gpsStatus(GPS_STATUS_ENGINE_OFF);
		s_controlThreadInfo.mainReady=true;
		pthread_cond_signal(&s_controlThreadInfo.threadMainReadyCond);
		pthread_mutex_unlock(&s_controlThreadInfo.threadMainReadyMutex);
		UBX_LOG(LCAT_DEBUG, "Signaling of readyness completed. Return normally and let ubx-thread continue");
	}
	return s_myIf.m_ready  ? 0 : 1;
}

int CGpsIf::start(void)
{
    UBX_LOG(LCAT_VERBOSE, "(%u):", (unsigned int) pthread_self());

	if (s_myIf.m_ready)
	{
		return controlThreadInfoSendCmd(&s_controlThreadInfo, CMD_START_SI) ? 0 : 1;
	}

	UBX_LOG(LCAT_ERROR, "Not initialised");
	return 1;
}

int CGpsIf::stop(void)
{
    UBX_LOG(LCAT_VERBOSE, "(%u):", (unsigned int) pthread_self());

	if (s_myIf.m_ready)
	{
		return controlThreadInfoSendCmd(&s_controlThreadInfo, CMD_STOP_SI) ? 0 : 1;
	}

	UBX_LOG(LCAT_ERROR, "Not initialised");
	return 1;
}

void CGpsIf::cleanup(void)
{
    UBX_LOG(LCAT_DEBUG, "(%u):", (unsigned int) pthread_self());

	if (s_myIf.m_ready)
	{
		controlThreadInfoSendCmd(&s_controlThreadInfo, CMD_STOP_SI);
	}
	else
	{
		UBX_LOG(LCAT_ERROR, "Not initialised");
	}
}

int CGpsIf::injectTime(GpsUtcTime timeGpsUtc, int64_t timeReference, int uncertainty)
{
	time_t tUtc = (long) (timeGpsUtc/1000);
	char s[20];
	struct tm t;
	strftime(s, 20, "%Y.%m.%d %H:%M:%S", gmtime_r(&tUtc, &t));

	UBX_LOG(LCAT_VERBOSE, "timeGpsUtc=%s.%03d timeReference=%lli uncertainty=%.3f ms",
				s, (int)(timeGpsUtc%1000), timeReference,uncertainty*0.001);

	if (s_myIf.m_ready)
	{
		gps_state_inject_time(timeGpsUtc, timeReference, uncertainty);
		return 0;
	}

	UBX_LOG(LCAT_ERROR, "Not initialised");
    return 1;
}

int CGpsIf::injectLocation(double latitude, double longitude, float accuracy)
{
    UBX_LOG(LCAT_VERBOSE, "latitude=%.6f longitude=%.6f accuracy=%.2f",
				latitude, longitude, accuracy);
	if (s_myIf.m_ready)
	{
		gps_state_inject_location(latitude, longitude, accuracy);
		return 0;
	}

	UBX_LOG(LCAT_ERROR, "Not initialised");
    return 1;
}

void CGpsIf::deleteAidingData(GpsAidingData flags)
{
    UBX_LOG(LCAT_DEBUG, "flags=0x%X", flags);
	if (s_myIf.m_ready)
	{
		gps_state_delete_aiding_data(flags);
	}
	else
	{
		UBX_LOG(LCAT_ERROR, "Not initialised");
	}
}

#if (PLATFORM_SDK_VERSION <= 8 /* <=2.2 */)
void CGpsIf::setFixFrequency(int frequency)
{
	UBX_LOG(LCAT_DEBUG, "(%u): frequency=%i",
			(unsigned int) pthread_self(),
            frequency);
	if (s_myIf.m_ready)
	{
		frequency = frequency ? frequency : 1000;
		gps_state_set_interval(frequency);
	}
	else
	{
		UBX_LOG(LCAT_ERROR, "Not initialised");
	}
}

int CGpsIf::setPositionMode(GpsPositionMode mode, int fix_frequency)
{
	UBX_LOG(LCAT_DEBUG, "(%u): mode=%i(%s) fix_frequency=%i",
			(unsigned int) pthread_self(),
            mode, _LOOKUPSTR(mode, GpsPositionMode),
            fix_frequency);
	int min_interval = fix_frequency;
#else // (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
int CGpsIf::setPositionMode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
			uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
	UBX_LOG(LCAT_DEBUG, "(%u): mode=%i(%s) recurrence=%i(%s) min_interval=%u preferred_accuracy=%u preferred_time=%u",
			(unsigned int) pthread_self(),
            mode, _LOOKUPSTR(mode, GpsPositionMode),
            recurrence, _LOOKUPSTR(recurrence, GpsPositionRecurrence),
            min_interval,
            preferred_accuracy,
            preferred_time);
#endif
	if (s_myIf.m_ready)
	{
		s_myIf.m_mode = mode;

		min_interval = min_interval ? min_interval : 1000;
		gps_state_set_interval(min_interval);
	}
	else
	{
		UBX_LOG(LCAT_ERROR, "Not initialised");
	}

    return 0;
}

const void* CGpsIf::getExtension(const char* name)
{
    UBX_LOG(LCAT_DEBUG, "name='%s'", name);
#if defined CDEBUGIF_EN
    if (!strcmp(name, GPS_DEBUG_INTERFACE))	return CDebugIf::getIf();
#endif
#ifdef SUPL_ENABLED
	if (!strcmp(name, AGPS_INTERFACE))		return CAgpsIf::getIf();
    if (!strcmp(name, GPS_NI_INTERFACE))	return CNiIf::getIf();
#endif
    if (!strcmp(name, AGPS_RIL_INTERFACE))	return CRilIf::getIf();
    if (!strcmp(name, GPS_XTRA_INTERFACE))	return CXtraIf::getIf();
    return NULL;
}

///////////////////////////////////////////////////////////////////////////////
// operations

void CGpsIf::gpsStatus(GpsStatusValue gpsStatusValue)
{
	if (!s_myIf.m_ready)
    {
        UBX_LOG(LCAT_ERROR, "class not initialized");
        return;
    }

	if (gpsStatusValue == s_myIf.m_lastStatusValue)
	{
		UBX_LOG(LCAT_VERBOSE, "current GPS status: (still) %d (%s)",
		     gpsStatusValue ,_LOOKUPSTR(gpsStatusValue, GpsStatusValue));
	}
	else
	{
		UBX_LOG(LCAT_VERBOSE, "GPS status change: %d (%s) => %d (%s)",
		     s_myIf.m_lastStatusValue,
		     _LOOKUPSTR(s_myIf.m_lastStatusValue, GpsStatusValue),
		     gpsStatusValue,
		     _LOOKUPSTR(gpsStatusValue, GpsStatusValue));
		s_myIf.m_lastStatusValue = gpsStatusValue;
		if (gpsStatusValue == GPS_STATUS_SESSION_END)
		{
			GpsSvStatus svStatus;
			memset(&svStatus, 0, sizeof(GpsSvStatus));
			IF_ANDROID23( svStatus.size = sizeof(GpsSvStatus); )
			s_myIf.m_callbacks.sv_status_cb(&svStatus);
		}
		GpsStatus gpsStatusVar;
		IF_ANDROID23( gpsStatusVar.size = sizeof(gpsStatusVar); )
		gpsStatusVar.status = gpsStatusValue;
		s_myIf.m_callbacks.status_cb(&gpsStatusVar);
	}
}

#if (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
void CGpsIf::nmea(const char* data, int length)
{
	if (!s_myIf.m_ready)
    {
        UBX_LOG(LCAT_ERROR, "class not initialized");
        return;
    }
	if (s_myIf.m_callbacks.nmea_cb != NULL)
	{
		struct timeval tv;
		gettimeofday(&tv, NULL);
		GpsUtcTime gpsUtcTime = (long long) tv.tv_sec * 1000 + (long long) tv.tv_usec / 1000;
		s_myIf.m_callbacks.nmea_cb(gpsUtcTime, data, length);
	}
}
#endif

#if (PLATFORM_SDK_VERSION >= 14 /* >=4.0 */)
void CGpsIf::requestUtcTime(void)
{
	if (!s_myIf.m_ready)
    {
        UBX_LOG(LCAT_ERROR, "class not initialized");
        return;
    }
	if (s_myIf.m_callbacks.request_utc_time_cb != NULL)
	{
		UBX_LOG(LCAT_VERBOSE, "");
		s_myIf.m_callbacks.request_utc_time_cb();
	}
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Debug / Testing support

#ifndef ANDROID_BUILD

extern "C" void endControlThread(void)
{
    UBX_LOG(LCAT_DEBUG, "Send thread exit command");
    bool ok = controlThreadInfoSendCmd(&s_controlThreadInfo, CMD_EXIT);
    pthread_join(s_mainControlThread, NULL);
	s_mainControlThread = NULL;
    UBX_LOG(LCAT_DEBUG, "Thread exited ok=%d", ok);
}
#endif

