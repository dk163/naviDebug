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
 * $Id: ubx_localDb.h 83728 2014-08-06 12:30:21Z fabio.robbiani $
 * $HeadURL:
 *http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_localDb.h
 *$
 *****************************************************************************/

#pragma once

#include "database.h"
#include "gps_thread.h"
#include <mutex>

///////////////////////////////////////////////////////////////////////////////

class CAndroidDatabase : public CDatabase
{
public:
  CAndroidDatabase();
  ~CAndroidDatabase();

  static CAndroidDatabase *getInstance();
  GpsUtcTime GetGpsUtcTime(void) const;

  virtual STATE_t Commit(bool bClear);

  void setEpochInterval(int timeIntervalMs, int64_t nextReportEpochMs);
  void setGpsState(ControlThreadInfo *pGpsState) { m_pGpsState = pGpsState; };
  int64_t getNextReportEpoch(void) const { return m_nextReportEpochMs; };
  int getAgc(void);
  void incPublish(void);
  void decPublish(void);
  void resetPublish(void) { m_publishCount = 0; };

protected:
  virtual void LockOutputDatabase(void);
  virtual void UnlockOutputDatabase(void);

  ControlThreadInfo *m_pGpsState{nullptr};
  // int64_t m_lastReportTime{time(NULL} * 1000}; // Debug
  // time interval delay
  std::mutex m_timeIntervalMutex{};
  int m_timeInterval{0};
  int64_t m_nextReportEpochMs{0};
  int m_publishCount{0}; /*!< Publishing off by default */

  enum SVID_SBAS_t
  {
    SVID_SBAS_ANDROID_MIN = 120,
    SVID_SBAS_UBLOX_MIN = 33,
    SVID_SBAS_UBLOX_MAX = 64,
  };

  enum SVID_GLO_t
  {
    SVID_GLO_ANDROID_MIN = 1,
    SVID_GLO_UBLOX_MIN = 65,
    SVID_GLO_UBLOX_MAX = 96,
  };

  enum SVID_GAL_t
  {
    SVID_GAL_UBLOX_MIN = 1,
    SVID_GAL_UBLOX_MAX = 36,
  };

  enum SVID_BDS_t
  {
    SVID_BDS_UBLOX_MIN = 1,
    SVID_BDS_UBLOX_MAX = 37,
  };

  typedef struct
  {
    float azimuth, elevation, snr;
    U1 gnss, svid;
    bool azimuthValid, elevationValid, snrValid, gnssValid, svidValid;
  } UBGnssSvInfo_t;

  void GetSvInfo(UBGnssSvInfo_t &info, int i) const;
  int16_t GetAndroidSvidOffset(const UBGnssSvInfo_t &info) const;

#if (PLATFORM_SDK_VERSION >= 24 /* >=7.0 */)
  GnssConstellationType GetAndroidConstellation(int gnssId) const;
  GnssSvFlags GetAndroidSvFlags(int navsta) const;
#else
  void SetAndroidSvFlags(GpsSvStatus *svStatus, int gnssId, int navsta, int prn) const;
#endif // (PLATFORM_SDK_VERSION >= 24 /* >=7.0 */)

  void CommitSvStatus(void) const;
  virtual bool GetCurrentTimestamp(TIMESTAMP &ft);

  template <typename T> bool getData(DATA_t data, T &v) const
  {
    if (data < DATA_NUM)
    {
      return m_varO[data].Get(v);
    }
    return false;
  }
};
