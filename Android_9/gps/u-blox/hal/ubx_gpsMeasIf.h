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
 *****************************************************************************/

#pragma once

#include "std_inc.h"
#include "ubx_messageDef.h"

#include "hidl/gps_ext.h"

class CGpsMeasIf
{
public:
  CGpsMeasIf();

  static const GpsMeasurementInterface *getIf(void);
  static CGpsMeasIf *getInstance(void);
  void navMeasurementUpdate(const UBX_RXM_RAWX &message);

private:
  static int init(GpsMeasurementCallbacks *callbacks);
  static void close();

  GnssMeasurementFlags getFlags(const UBX_RXM_RAWX_INDIVIDUAL &measurement);
  GnssMultipathIndicator getMultipathIndicator(const UBX_RXM_RAWX_INDIVIDUAL &measurement);
  bool fillGnssData(GnssData &gnssData, const UBX_RXM_RAWX &message);
#if (PLATFORM_SDK_VERSION >= 26 /* >=8.0 */)
  bool fillGnssDataExt(GnssDataExt &gnssDataExt);
#endif
  bool fillClockData(GnssClock &clock, const UBX_RXM_RAWX &message);
  unsigned int fillMeasurementData(GnssMeasurement *measurements, const UBX_RXM_RAWX &message);
  static bool haveGpsCallback();
  static bool haveGnssCallback();
  static bool noCallbacksDefined();
  static void initializeCallbacks(const GpsMeasurementCallbacks &callbacks);
  static void clearCallbacks();
  static uint32_t computeClkResetCount(const UBX_RXM_RAWX_SUMMARY &summary);
  static void resetClkResetCount();

  void setClockDrift(const double drift);
  void setClockUncertainty(double uncertainty);

  void initializeInternalClock(const UBX_RXM_RAWX_SUMMARY &rawMessageSummary);
  void setInternalClockBias(uint64_t bias);
  uint64_t getInternalClockBias() const;
  uint64_t computeGnssClockTime(uint64_t timeNow) const;
  uint64_t computeGnssClockBias(uint64_t timeNow) const;
  bool internalClockBiasInitialized() const;

  // interface
  static const GpsMeasurementInterface s_gps_meas_interface;
  static uint32_t m_clockResetCount;
  GpsMeasurementCallbacks m_gps_meas_callbacks{};
  uint64_t gpsInternalClockBias_{0}; //! Module internal clock bias
  double clockDrift_{0};
  double clockUncertainty_{0};
};
