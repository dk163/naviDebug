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
 * $HeadURL:
 *http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_moduleIf.cpp
 *$
 *****************************************************************************/

/*!
  \file
  \brief Defines the interface to the Android Framework

  Defines the interface of the HAL module to the Android Framework and
  provides the functions to start it up, shut it down, inject
  information and others.
  */

#include <errno.h>
#include <math.h>
#include <new>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#ifndef ANDROID_BUILD
// Needed for Linux build
#include <malloc.h>
#endif

#include "std_types.h"
#include "ubx_log.h"

#ifdef SUPL_ENABLED
#include "ubx_agpsIf.h"
#include "ubx_niIf.h"
#endif
#include "ubx_moduleIf.h"
#include "ubx_rilIf.h"
#include "ubx_xtraIf.h"
#ifdef RAW_MEAS_ENABLED
#include "ubx_gnssConfIf.h"
#include "ubx_gpsMeasIf.h"
#include "ubx_gpsNavMesIf.h"
#endif
#include "gps_thread.h"

#include "version.h"
// For GetVersion() of libParser
#include "parserbuffer.h"
// For mgaGetVersion() of libMGA
#include "agnss/mga/src/libMga.h"
#ifdef ANDROID_BUILD
#include "ubx_cfg.h"
#include <ubx_androidHelper.h>
#endif

using namespace ublox::log::stringtables;

#define NOTINITIALIZEDCHECK(RETVAL)                                                                \
  if (!StaticObjectInitialized())                                                                  \
  {                                                                                                \
    UBX_LOG(LCAT_ERROR, "Not initialised");                                                        \
    return RETVAL;                                                                                 \
  }

namespace
{
  static const uint16_t DEFAULT_YEAR_OF_HW = 2015;      // Android Year of Hardware
  static const uint16_t YEAR_OF_HW_GENERATION_9 = 2017; // Android Year of
                                                        // Hardware
  uint16_t getYearOfHw()
  {
    CCfg cfg;
    cfg.load(Android::UBLOX_CONFIG_FILE.c_str());

    if (cfg.get("RECEIVER_GENERATION", 0) >= 9)
      return YEAR_OF_HW_GENERATION_9;

    return DEFAULT_YEAR_OF_HW;
  }

  const std::string GetVersion(void) { return ANDROID_GNSS_DRIVER_VERSION; }
}
static ControlThreadInfo s_controlThreadInfo;
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
  .tag = HARDWARE_MODULE_TAG,            // uint32_t
  .version_major = 2,                    // uint16_t
  .version_minor = 0,                    // uint16_t
  .id = GPS_HARDWARE_MODULE_ID,          // const char *
  .name = "u-blox GPS/GNSS library",     // const char *
  .author = "u-blox AG - Switzerland",   // const char *
  .methods = &CGpsIf::s_hwModuleMethods, // struct hw_module_methods_t *
  .dso = NULL,                           // module's dso
  .reserved = {0}                        // uint32_t *, padding
};

struct hw_module_methods_t CGpsIf::s_hwModuleMethods = {
  .open = CGpsIf::hwModuleOpen // open a specific device
};

int CGpsIf::hwModuleOpen(const struct hw_module_t *module,
                         char const *name,
                         struct hw_device_t **device)
{
  ((void)(name));
  struct gps_device_t *dev = new (std::nothrow) gps_device_t{};

  if (!dev)
    return 1;

  dev->common.tag = HARDWARE_DEVICE_TAG;
  dev->common.version = 0;
  dev->common.module = const_cast<struct hw_module_t *>(module);
  dev->common.close = CGpsIf::hwModuleClose;
  dev->get_gps_interface = CGpsIf::getIf;
  *device = (struct hw_device_t *)(void *)dev;

  return 0;
}

int CGpsIf::hwModuleClose(struct hw_device_t *device)
{
  delete device;
  return 0;
}
#else // (PLATFORM_SDK_VERSION <= 8 /* <=2.2 */)
const GpsInterface *gps_get_interface() { return CGpsIf::getIf(NULL); }
#endif

/*******************************************************************************
 * INTERFACE
 ******************************************************************************/

static CGpsIf s_myIf;

const GpsInterface CGpsIf::s_interface = {
  IF_ANDROID23(.size = sizeof(GpsInterface), ).init = CGpsIf::init,
  .start = CGpsIf::start,
  .stop = CGpsIf::stop,
#if (PLATFORM_SDK_VERSION <= 8 /* <=2.2 */)
  .set_fix_frequency = CGpsIf::setFixFrequency,
#endif
  .cleanup = CGpsIf::cleanup,
  .inject_time = CGpsIf::injectTime,
  IF_ANDROID23(.inject_location = CGpsIf::injectLocation, ).delete_aiding_data =
    CGpsIf::deleteAidingData,
  .set_position_mode = CGpsIf::setPositionMode,
  .get_extension = CGpsIf::getExtension,
};

CGpsIf::CGpsIf() {}

const GpsInterface *CGpsIf::getIf(struct gps_device_t * /*dev*/) { return &s_interface; }

CGpsIf *CGpsIf::getInstance() { return &s_myIf; }

GpsPositionMode CGpsIf::getMode(void) const { return m_mode; };

uint32_t CGpsIf::getCapabilities(void) const { return m_capabilities; };

#if (PLATFORM_SDK_VERSION <= 8 /* <=2.2 */)
extern "C" void *CGpsIfThread22(void *info)
{
  ubx_thread(info);
  pthread_exit(NULL);
  return NULL;
}
#endif

int CGpsIf::init(GpsCallbacks *callbacks)
{
  if (StaticObjectInitialized())
  {
    UBX_LOG(LCAT_ERROR, "already initialized");
    return 0; // Report success since we are already initialised
  }

  outputVersionInformation();

#if (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
  initializeGpsCallbacks(*callbacks);
#endif

  UBX_LOG(LCAT_DEBUG, "(%u): Initializing - pid %i", (unsigned int)pthread_self(), getpid());

#if (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
  prepareCapabilities();
  informAndroidAboutCapabilities();
  informAndroidAboutSystemInfo();
#endif
  createGpsThread();
  // Wait for ubx-thread to signal that the setup has been finished
  // and it is ready to run
  waitUbxThread();

  UBX_LOG(LCAT_DEBUG,
          "Initialized complete: result %i. ubx-thread %sready",
          StaticObjectInitialized(),
          StaticObjectInitialized() ? "" : "NOT ");
  if (!StaticObjectInitialized())
  {
    // Init failed -  release resources
    UBX_LOG(LCAT_ERROR, "Initialisation failed! Freeing the ressources");
    controlThreadInfoRelease(&s_controlThreadInfo);
  }
  else // If everything is fine, tell ubx_thread to continue
  {
    // Tell the ubx-thread that the main-thread is ready
    informMainReadyToUbx();
  }
  return StaticObjectInitialized() ? 0 : 1;
}

int CGpsIf::start(void)
{
  UBX_LOG(LCAT_VERBOSE, "(%u):", (unsigned int)pthread_self());

  NOTINITIALIZEDCHECK(1);

  return controlThreadInfoSendCmd(&s_controlThreadInfo, CMD_START_SI) ? 0 : 1;
}

int CGpsIf::stop(void)
{
  UBX_LOG(LCAT_VERBOSE, "(%u):", (unsigned int)pthread_self());

  NOTINITIALIZEDCHECK(1);

  return controlThreadInfoSendCmd(&s_controlThreadInfo, CMD_STOP_SI) ? 0 : 1;
}

void CGpsIf::cleanup(void)
{
  UBX_LOG(LCAT_DEBUG, "(%u):", (unsigned int)pthread_self());

  controlThreadInfoSendCmd(&s_controlThreadInfo, CMD_STOP_SI);
}

int CGpsIf::injectTime(GpsUtcTime timeGpsUtc, int64_t timeReference, int uncertainty)
{
  time_t tUtc = (long)(timeGpsUtc / 1000);
  char s[20];
  struct tm t;
  strftime(s, 20, "%Y.%m.%d %H:%M:%S", gmtime_r(&tUtc, &t));

  UBX_LOG(LCAT_VERBOSE,
          "timeGpsUtc=%s.%03d timeReference=%lli uncertainty=%.3f ms",
          s,
          (int)(timeGpsUtc % 1000),
          timeReference,
          uncertainty * 0.001);

  NOTINITIALIZEDCHECK(1);
  gps_state_inject_time(timeGpsUtc, timeReference, uncertainty);
  return 0;
}

int CGpsIf::injectLocation(double latitude, double longitude, float accuracy)
{
  UBX_LOG(
    LCAT_VERBOSE, "latitude=%.6f longitude=%.6f accuracy=%.2f", latitude, longitude, accuracy);
  NOTINITIALIZEDCHECK(1);
  gps_state_inject_location(latitude, longitude, accuracy);
  return 0;
}

void CGpsIf::deleteAidingData(GpsAidingData flags)
{
  UBX_LOG(LCAT_DEBUG, "flags=0x%X", flags);
  gps_state_delete_aiding_data(flags);
}

#if (PLATFORM_SDK_VERSION <= 8 /* <=2.2 */)
void CGpsIf::setFixFrequency(int frequency)
{
  UBX_LOG(LCAT_DEBUG, "(%u): frequency=%i", (unsigned int)pthread_self(), frequency);
  frequency = frequency ? frequency : 1000;
  gps_state_set_interval(frequency);
}

int CGpsIf::setPositionMode(GpsPositionMode mode, int fix_frequency)
{
  UBX_LOG(LCAT_DEBUG,
          "(%u): mode=%i(%s) fix_frequency=%i",
          (unsigned int)pthread_self(),
          mode,
          _LOOKUPSTR(mode, GpsPositionMode),
          fix_frequency);
  int min_interval = fix_frequency;
#else // (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
int CGpsIf::setPositionMode(GpsPositionMode mode,
                            GpsPositionRecurrence recurrence,
                            uint32_t min_interval,
                            uint32_t preferred_accuracy,
                            uint32_t preferred_time)
{
  UBX_LOG(LCAT_DEBUG,
          "(%u): mode=%i(%s) recurrence=%i(%s) min_interval=%u "
          "preferred_accuracy=%u preferred_time=%u",
          (unsigned int)pthread_self(),
          mode,
          _LOOKUPSTR(mode, GpsPositionMode),
          recurrence,
          _LOOKUPSTR(recurrence, GpsPositionRecurrence),
          min_interval,
          preferred_accuracy,
          preferred_time);
#endif
  if (!StaticObjectInitialized())
  {
    UBX_LOG(LCAT_ERROR, "Not initialised");
  }
  else
  {
    s_myIf.m_mode = mode;

    min_interval = min_interval ? min_interval : 1000;
    gps_state_set_interval(min_interval);
  }

  return 0;
}

const void *CGpsIf::getExtension(const char *name)
{
  UBX_LOG(LCAT_DEBUG, "Get Extension of name='%s'", name);
#ifdef RAW_MEAS_ENABLED
  if (!strcmp(name, GPS_MEASUREMENT_INTERFACE))
    return CGpsMeasIf::getIf();
  if (!strcmp(name, GNSS_CONFIGURATION_INTERFACE))
    return CGnssConfIf::getIf();
  if (!strcmp(name, GPS_NAVIGATION_MESSAGE_INTERFACE))
    return CGpsNavMesIf::getIf();
#endif
#if defined CDEBUGIF_EN
  if (!strcmp(name, GPS_DEBUG_INTERFACE))
    return CDebugIf::getIf();
#endif
#ifdef SUPL_ENABLED
  if (!strcmp(name, AGPS_INTERFACE))
    return CAgpsIf::getIf();
  if (!strcmp(name, GPS_NI_INTERFACE))
    return CNiIf::getIf();
#endif
  if (!strcmp(name, AGPS_RIL_INTERFACE))
    return CRilIf::getIf();
  if (!strcmp(name, GPS_XTRA_INTERFACE))
    return CXtraIf::getIf();
  return NULL;
}

///////////////////////////////////////////////////////////////////////////////
// operations

void CGpsIf::gpsStatus(GpsStatusValue gpsStatusValue)
{
  NOTINITIALIZEDCHECK();

  if (gpsStatusValue == s_myIf.m_lastStatusValue)
  {
    UBX_LOG(LCAT_VERBOSE,
            "current GPS status: (still) %d (%s)",
            gpsStatusValue,
            _LOOKUPSTR(gpsStatusValue, GpsStatusValue));
  }
  else
  {
    UBX_LOG(LCAT_VERBOSE,
            "GPS status change: %d (%s) => %d (%s)",
            s_myIf.m_lastStatusValue,
            _LOOKUPSTR(s_myIf.m_lastStatusValue, GpsStatusValue),
            gpsStatusValue,
            _LOOKUPSTR(gpsStatusValue, GpsStatusValue));
    s_myIf.m_lastStatusValue = gpsStatusValue;
    if (gpsStatusValue == GPS_STATUS_SESSION_END)
    {
      GpsSvStatus svStatus{};
      IF_ANDROID23(svStatus.size = sizeof(GpsSvStatus);)
      s_myIf.m_callbacks.sv_status_cb(&svStatus);
    }
    GpsStatus gpsStatusVar;
    IF_ANDROID23(gpsStatusVar.size = sizeof(gpsStatusVar);)
    gpsStatusVar.status = gpsStatusValue;
    s_myIf.m_callbacks.status_cb(&gpsStatusVar);
  }
}

#if (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
void CGpsIf::nmea(const char *data, int length)
{
  NOTINITIALIZEDCHECK();

  if (s_myIf.m_callbacks.nmea_cb != NULL)
  {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    GpsUtcTime gpsUtcTime = (long long)tv.tv_sec * 1000 + (long long)tv.tv_usec / 1000;
    s_myIf.m_callbacks.nmea_cb(gpsUtcTime, data, length);
  }
}
#endif

#if (PLATFORM_SDK_VERSION >= 14 /* >=4.0 */)
void CGpsIf::requestUtcTime(void)
{
  NOTINITIALIZEDCHECK();

  if (s_myIf.m_callbacks.request_utc_time_cb != NULL)
  {
    UBX_LOG(LCAT_VERBOSE, "");
    s_myIf.m_callbacks.request_utc_time_cb();
  }
}
#endif

bool CGpsIf::StaticObjectInitialized() { return s_myIf.m_ready; }

void CGpsIf::outputVersionInformation()
{
  UBX_LOG(LCAT_VERBOSE, "Driver version %s", GetVersion().c_str());
  UBX_LOG(LCAT_VERBOSE, "libMGA version %s", mgaGetVersion());
  UBX_LOG(LCAT_VERBOSE, "libParser version %s", CParserBuffer::GetVersion());
}
void CGpsIf::initializeGpsCallbacks(GpsCallbacks &callbacks)
{
  if (callbacks.size == sizeof(GpsCallbacks))
  {
    s_myIf.m_callbacks = callbacks;
  }
  else
  {
    UBX_LOG(LCAT_WARNING, "callback size %zd != %zd", callbacks.size, sizeof(GpsCallbacks));
  }
}

void CGpsIf::prepareCapabilities()
{
  s_myIf.m_capabilities = GPS_CAPABILITY_SCHEDULING;
#if (PLATFORM_SDK_VERSION >= 14 /* >=4.0 */)
  s_myIf.m_capabilities |=
    GPS_CAPABILITY_ON_DEMAND_TIME | GPS_CAPABILITY_MEASUREMENTS | GPS_CAPABILITY_NAV_MESSAGES;
#endif
#ifdef SUPL_ENABLED
  s_myIf.m_capabilities |= GPS_CAPABILITY_MSB | GPS_CAPABILITY_MSA;
#endif
  UBX_LOG(LCAT_VERBOSE,
          "set_capabilities=%d(%s)",
          s_myIf.m_capabilities,
          _LOOKUPSTRX(s_myIf.m_capabilities, GpsCapabilityFlags));
}

void CGpsIf::informAndroidAboutCapabilities()
{
  s_myIf.m_callbacks.set_capabilities_cb(s_myIf.m_capabilities);
}

void CGpsIf::informAndroidAboutSystemInfo()
{
#ifdef RAW_MEAS_ENABLED
  GnssSystemInfo gnssSystemInfo;
  gnssSystemInfo.size = sizeof(GnssSystemInfo);
  gnssSystemInfo.year_of_hw = getYearOfHw();
  s_myIf.m_callbacks.set_system_info_cb(&(gnssSystemInfo));
#endif
}

void CGpsIf::createGpsThread()
{
#if (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
  s_mainControlThread =
    s_myIf.m_callbacks.create_thread_cb("gps thread", ubx_thread, &s_controlThreadInfo);
#else // (PLATFORM_SDK_VERSION <= 8 /* <=2.2 */)
  s_mainControlThread = (pthread_t)NULL;
  pthread_create(&s_mainControlThread, NULL, CGpsIfThread22, &s_controlThreadInfo);
#endif
}

void CGpsIf::waitUbxThread()
{
  UBX_LOG(LCAT_DEBUG, "Wait for ubx-thread to get ready");
  std::unique_lock<std::mutex> lock(s_controlThreadInfo.threadCmdCompleteMutex);
  s_controlThreadInfo.threadCmdCompleteCond.wait(lock,
                                                 [] { return s_controlThreadInfo.cmdResult; });
  s_myIf.m_ready = (s_controlThreadInfo.cmdResult == 1);
  lock.unlock();
}

void CGpsIf::informMainReadyToUbx()
{
  UBX_LOG(LCAT_DEBUG, "Signal to the waiting ubx-thread that main-thread is ready");
  {
    std::lock_guard<std::mutex> guard(s_controlThreadInfo.threadMainReadyMutex);
    gpsStatus(GPS_STATUS_ENGINE_OFF);
    s_controlThreadInfo.mainReady = true;
  }
  s_controlThreadInfo.threadMainReadyCond.notify_all();
  UBX_LOG(LCAT_DEBUG,
          "Signaling of readyness completed. Return normally and "
          "let ubx-thread continue");
}

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
