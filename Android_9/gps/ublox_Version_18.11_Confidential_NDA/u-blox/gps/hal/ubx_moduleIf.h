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
 * $Id$
 * $HeadURL$
 *****************************************************************************/

#pragma once

#include "std_inc.h"

///////////////////////////////////////////////////////////////////////////////

class CGpsIf
{
public:
  CGpsIf();
  static struct hw_module_methods_t s_hwModuleMethods;
  static const GpsInterface *getIf(struct gps_device_t * /*dev*/);

  static CGpsIf *getInstance(void);

  GpsPositionMode getMode(void) const;
  uint32_t getCapabilities(void) const;

  // operations
  static void gpsStatus(GpsStatusValue gpsStatusValue);
  static void nmea(const char *data, int length);
#if (PLATFORM_SDK_VERSION >= 14 /* =4.0 */)
  static void requestUtcTime(void);
#endif
#if (PLATFORM_SDK_VERSION >= 24 /* >=7.0.0*/)
  static void setGnssSystemInfo(uint16_t year_of_hw);
#endif // if (PLATFORM_SDK_VERSION >=24 /* >=7.0.0*/)
  GpsCallbacks m_callbacks{};

private:
  // interface hw module
  static int hwModuleOpen(const struct hw_module_t *module, char const *name,
                          struct hw_device_t **device);
  static int hwModuleClose(struct hw_device_t *device);

  // interface
  static int init(GpsCallbacks *callbacks);
  static int start(void);
  static int stop(void);
  static void cleanup(void);
  static int injectTime(GpsUtcTime timeGpsUtc, int64_t timeReference,
                        int uncertainty);
  static int injectLocation(double latitude, double longitude, float accuracy);
  static void deleteAidingData(GpsAidingData f);
#if (PLATFORM_SDK_VERSION > 8 /* >2.2 */)
  static int setPositionMode(GpsPositionMode mode,
                             GpsPositionRecurrence recurrence,
                             uint32_t min_interval, uint32_t preferred_accuracy,
                             uint32_t preferred_time);
#else
  static void setFixFrequency(int frequency);
  static int setPositionMode(GpsPositionMode mode, int fix_frequency);
#endif
  static const void *getExtension(const char *name);
  static bool StaticObjectInitialized();
  static void outputVersionInformation();
  static void initializeGpsCallbacks(GpsCallbacks &callbacks);
  static void prepareCapabilities();
  static void informAndroidAboutCapabilities();
  static void informAndroidAboutSystemInfo();
  static void createGpsThread();
  static void waitUbxThread();
  static void informMainReadyToUbx();

  // variables
  static const GpsInterface s_interface;
  bool m_ready{false};
  GpsPositionMode m_mode{GPS_POSITION_MODE_MS_BASED};
  GpsStatusValue m_lastStatusValue{GPS_STATUS_NONE};
  uint32_t m_capabilities{};
};
