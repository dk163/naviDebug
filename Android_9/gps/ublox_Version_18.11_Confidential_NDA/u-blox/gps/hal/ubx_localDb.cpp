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

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "std_lang_def.h"
#include "std_macros.h"
#include "std_types.h"
#include "ubx_log.h"
#include "ubx_timer.h"
#include "ubx_units.h"

#include "ubx_localDb.h"

#include "hidl/gps_ext.h"

namespace
{
  using namespace ublox::Units;
  unsigned long convertToSeconds(tm &time)
  {
    return time.tm_sec + 1.0_minutes * time.tm_min + 1.0_hours * time.tm_hour +
           1.0_days * time.tm_yday + 1.0_years * time.tm_year;
  }
}

static CAndroidDatabase s_database;

CAndroidDatabase::CAndroidDatabase() : CDatabase() {}

CAndroidDatabase::~CAndroidDatabase() { m_pGpsState = NULL; }

CAndroidDatabase *CAndroidDatabase::getInstance() { return &s_database; }

GpsUtcTime CAndroidDatabase::GetGpsUtcTime(void) const
{
  TIMESTAMP ts;
  if (getData(DATA_UTC_TIMESTAMP, ts))
  {
    struct tm ti
    {
    };
    ti.tm_year = ts.wYear - 1900;
    ti.tm_mon = ts.wMonth - 1;
    ti.tm_mday = ts.wDay;
    ti.tm_hour = ts.wHour;
    ti.tm_min = ts.wMinute;
    ti.tm_sec = (int)(ts.lMicroseconds / 1000000);
    unsigned long us = (unsigned long)(ts.lMicroseconds - (unsigned long)ti.tm_sec * 1000000);
    ti.tm_isdst = -1;

    time_t t = mktime(&ti);
    //            UBX_LOG(LCAT_VERBOSE, "%s", ctime(&t));

    // calc utc / local difference
    time_t now = time(NULL);
    struct tm tmLocal;
    struct tm tmUtc;
    long timeLocal, timeUtc;

    gmtime_r(&now, &tmUtc);
    localtime_r(&now, &tmLocal);
    timeLocal = convertToSeconds(tmLocal);
    timeUtc = convertToSeconds(tmUtc);
    long utcDiff = timeUtc - timeLocal;
    //            UBX_LOG(LCAT_VERBOSE, "Time utcDiff: %li", utcDiff);

    t -= utcDiff;
    //            UBX_LOG(LCAT_VERBOSE, "%s", ctime(&t));

    GpsUtcTime gpsUtcTime = (GpsUtcTime)t * (GpsUtcTime)1000 + (GpsUtcTime)(us / 1000);

    return gpsUtcTime;
  }
  return 0;
}

void CAndroidDatabase::GetSvInfo(UBGnssSvInfo_t &info, int i) const
{
  // azimuth
  info.azimuthValid = getData(DATA_SATELLITES_IN_VIEW_AZIMUTH_(i), info.azimuth);
  if (info.azimuthValid)
  {
    info.azimuthValid = info.azimuth >= 0.0f && info.azimuth <= 360.0f;
  }

  // elevation
  info.elevationValid = getData(DATA_SATELLITES_IN_VIEW_ELEVATION_(i), info.elevation);
  if (info.elevationValid)
  {
    info.elevationValid = info.elevation >= 0.0f && info.elevation <= 90.0f;
  }

  // snr
  info.snrValid = getData(DATA_SATELLITES_IN_VIEW_STN_RATIO_(i), info.snr);
  if (info.snrValid)
  {
    info.snrValid = info.snr > 0.0f;
  }

  // gnss id
  info.gnssValid = getData(DATA_SATELLITES_IN_VIEW_GNSSID_(i), info.gnss);
  if (info.gnssValid)
  {
    info.gnssValid = info.gnss >= GNSSID_GPS && info.gnss <= GNSSID_GLO;
  }

  // svid
  info.svidValid = getData(DATA_SATELLITES_IN_VIEW_SVID_(i), info.svid);
  if (info.svidValid)
  {
    info.svidValid = info.svid == 0;
  }
}

#if (PLATFORM_SDK_VERSION >= 24 /* >=7.0 */)

int16_t CAndroidDatabase::GetAndroidSvidOffset(const UBGnssSvInfo_t &info) const
{
  // Translate NMEA id to (android's) svid
  // See hardware/gps.h for expected values.
  if (info.gnss == GNSSID_SBAS && info.svid >= SVID_SBAS_UBLOX_MIN &&
      info.svid <= SVID_SBAS_UBLOX_MAX)
  {
    return SVID_SBAS_ANDROID_MIN - SVID_SBAS_UBLOX_MIN;
  }
  else if (info.gnss == GNSSID_GLO && info.svid >= SVID_GLO_UBLOX_MIN &&
           info.svid <= SVID_GLO_UBLOX_MAX)
  {
    return SVID_GLO_ANDROID_MIN - SVID_GLO_UBLOX_MIN;
  }

  return 0;
}

GnssConstellationType CAndroidDatabase::GetAndroidConstellation(int gnssId) const
{
  switch (gnssId)
  {
  case GNSSID_GPS:
    return GNSS_CONSTELLATION_GPS;
  case GNSSID_SBAS:
    return GNSS_CONSTELLATION_SBAS;
  case GNSSID_GAL:
    return GNSS_CONSTELLATION_GALILEO;
  case GNSSID_BDS:
    return GNSS_CONSTELLATION_BEIDOU;
  case GNSSID_QZSS:
    return GNSS_CONSTELLATION_QZSS;
  case GNSSID_GLO:
    return GNSS_CONSTELLATION_GLONASS;
  default:
    return GNSS_CONSTELLATION_UNKNOWN;
  }
}

GnssSvFlags CAndroidDatabase::GetAndroidSvFlags(int navsta) const
{
  GnssSvFlags flags = 0;

  if (navsta & IS_USED)
  {
    flags |= GNSS_SV_FLAGS_USED_IN_FIX;
  }

  if (navsta & HAS_EPH)
  {
    flags |= GNSS_SV_FLAGS_HAS_EPHEMERIS_DATA;
  }

  if (navsta & HAS_ALM)
  {
    flags |= GNSS_SV_FLAGS_HAS_ALMANAC_DATA;
  }

  return flags;
}

#else

int16_t CAndroidDatabase::GetAndroidSvidOffset(const UBGnssSvInfo_t &info) const
{
  // The Galileo and GPS SV IDs overlap, map Galileo to 101-136
  if (info.gnss == GNSSID_GAL && info.svid >= SVID_GAL_UBLOX_MIN && info.svid <= SVID_GAL_UBLOX_MAX)
  {
    return 100;
  }

  // The BeiDou and GPS SV IDs overlap, map BeiDou to 201-237
  if (info.gnss == GNSSID_BDS && info.svid >= SVID_BDS_UBLOX_MIN && info.svid <= SVID_BDS_UBLOX_MAX)
  {
    return 200;
  }

  return 0;
}

void CAndroidDatabase::SetAndroidSvFlags(GpsSvStatus *svStatus,
                                         int gnssId,
                                         int navsta,
                                         int prn) const
{
  // Currently it is only possible to report availability
  // of ALM or EPH for GPS
  if (gnssId == GNSSID_GPS)
  {
    uint32_t prn_bit = 1UL << (prn - 1);
    if (navsta & IS_USED)
    {
      svStatus->used_in_fix_mask |= prn_bit;
    }

    if (navsta & HAS_EPH)
    {
      svStatus->ephemeris_mask |= prn_bit;
    }

    if (navsta & HAS_ALM)
    {
      svStatus->almanac_mask |= prn_bit;
    }
  }
}

#endif // (PLATFORM_SDK_VERSION >= 24 /* >=7.0 */)

void CAndroidDatabase::CommitSvStatus() const
{
  int i;
#if (PLATFORM_SDK_VERSION >= 24 /* >=7.0 */)
  GnssSvStatus svStatus = {};
#else
  GpsSvStatus svStatus = {};
#endif

  // Satellite status
  IF_ANDROID23(svStatus.size = sizeof(svStatus);)

  int numSvInDb = 0;
  if (!getData(DATA_SATELLITES_IN_VIEW, numSvInDb) || (numSvInDb <= 0))
  {
    return;
  }

  // Satellites in view
  int numValidSv = 0;
  int maxSvs = 0;

#if (PLATFORM_SDK_VERSION >= 24 /* >=7.0 */)
  maxSvs = GNSS_MAX_SVS;
#else
  maxSvs = GPS_MAX_SVS;
#endif

  for (i = 0; (i < numSvInDb) && (numValidSv < maxSvs); i++)
  {
    UBGnssSvInfo_t ubInfo;
    GetSvInfo(ubInfo, i);

    if (!ubInfo.azimuthValid &&   /* azimuth is undefined or invalid */
        !ubInfo.elevationValid && /* elevation is undefined or invalid */
        !ubInfo.snrValid && /* The signal to noise ratio is undefined or invalid
                               */
        /* The svid is not known for this satellite or type is invalid */
        (!ubInfo.svidValid || !ubInfo.gnssValid))
    {
      continue;
    }

    int svidOffset = GetAndroidSvidOffset(ubInfo);
    int navsta;
    bool navstaOk = false;

#if (PLATFORM_SDK_VERSION >= 24 /* >=7.0 */)

    GnssSvInfo *info = &svStatus.gnss_sv_list[numValidSv];
    info->size = sizeof(GnssSvInfo);
    info->azimuth = ubInfo.azimuthValid ? ubInfo.azimuth : -1;
    info->elevation = ubInfo.elevationValid ? ubInfo.elevation : -1;
    info->c_n0_dbhz = ubInfo.snrValid ? ubInfo.snr : -1;
    info->svid = ubInfo.svid + svidOffset;
    info->constellation = GetAndroidConstellation(ubInfo.gnss);

    navstaOk = getData(DATA_UBX_SATELLITES_IN_VIEW_NAV_STA_(i), navsta);
    if (navstaOk)
    {
      info->flags |= GetAndroidSvFlags(navsta);
    }

#else  // (PLATFORM_SDK_VERSION >= 24 /* >=7.0 */)

    GpsSvInfo *info = &svStatus.sv_list[numValidSv];
    IF_ANDROID23(info->size = sizeof(GpsSvInfo);)

    info->prn = ubInfo.svid + svidOffset;
    info->azimuth = ubInfo.azimuthValid ? ubInfo.azimuth : -1;
    info->elevation = ubInfo.elevationValid ? ubInfo.elevation : -1;
    info->snr = ubInfo.snrValid ? ubInfo.snr : -1;

    // For GPS SVs additional information can be provided to the framework
    navstaOk = getData(DATA_UBX_SATELLITES_IN_VIEW_NAV_STA_(i), navsta);
    if (navstaOk)
    {
      SetAndroidSvFlags(&svStatus, ubInfo.gnss, navsta, ubInfo.svid);
    }
#endif // (PLATFORM_SDK_VERSION >= 24 /* >=7.0 */)

#ifdef UBX_SV_STATUS_LOGGING
    if (navstaOk)
    {
      UBX_LOG(LCAT_VERBOSE,
              "SV: %i:%u -> %u status: %c%c%c",
              ubInfo.gnss,
              ubInfo.svid,
              ubInfo.svid + svidOffset,
              navsta & IS_USED ? 'U' : 'u',
              navsta & HAS_EPH ? 'E' : 'e',
              navsta & HAS_ALM ? 'A' : 'a');
    }
#endif // UBX_SV_STATUS_LOGGING

    numValidSv++;
  }

  // Report SV data to the framework
  if (numValidSv)
  {
    svStatus.num_svs = numValidSv;

#if (PLATFORM_SDK_VERSION >= 24 /* >=7.0 */)
    CGpsIf::getInstance()->m_callbacks.gnss_sv_status_cb(&svStatus);
#else
    CGpsIf::getInstance()->m_callbacks.sv_status_cb(&svStatus);
#endif

#ifdef UBX_SV_STATUS_LOGGING
    UBX_LOG(LCAT_VERBOSE, "SV status updated!");
#endif // UBX_SV_STATUS_LOGGING
  }
}

CDatabase::STATE_t CAndroidDatabase::Commit(bool bClear)
{
  CDatabase::STATE_t state;

  // Store commit time in database
  state = CDatabase::Commit(bClear);

  // UBX_LOG(LCAT_VERBOSE, "Perform commit: clear %i   state %i", bClear,
  // state);

  // UBX_LOG(LCAT_VERBOSE, "*** Epoch *** Now %lli, Last %lli, Interval %lli",
  //    now, lastReportTime, reportInterval);

  // lastReportTime = time(NULL) * 1000; // Debug

  int report = false;
  {
    std::lock_guard<std::mutex> guard(m_timeIntervalMutex);
    // UBX_LOG(LCAT_VERBOSE, "Ready to report");
    if (m_nextReportEpochMs)
    {
      int64_t monotonicNow = getMonotonicMsCounter();
      if (monotonicNow >= m_nextReportEpochMs)
      {
        report = true; // Yes we can report
        m_nextReportEpochMs = monotonicNow + m_timeInterval;
        // UBX_LOG(LCAT_VERBOSE, "Next timer epoch (%lli)",
        // m_nextReportEpochMs);
      }
    }
    else
    {
      report = true; // Yes we can report
                     // UBX_LOG(LCAT_VERBOSE, "Normal report epoch");
    }
  }

  if ((m_pGpsState != NULL) && (m_pGpsState->gpsState == GPS_STARTED) && (report))
  {
    // Driver in the 'started' state, so ok to report to framework
    // UBX_LOG(LCAT_VERBOSE, ">>> Reporting (%p)", m_pGpsState->pGpsInterface);
    // Location
    if (CGpsIf::getInstance()->m_callbacks.location_cb)
    {
      GpsLocation loc{};
      IF_ANDROID23(loc.size = sizeof(GpsLocation);)
      // position
      if (state == STATE_READY)
      {
        // UBX_LOG(LCAT_VERBOSE, ">>> Reporting Pos, Alt & Acc");
        if (getData(DATA_LASTGOOD_LATITUDE_DEGREES, loc.latitude) &&
            getData(DATA_LASTGOOD_LONGITUDE_DEGREES, loc.longitude))
          loc.flags |= GPS_LOCATION_HAS_LAT_LONG;
        if (getData(DATA_LASTGOOD_ALTITUDE_ELLIPSOID_METERS, loc.altitude))
          loc.flags |= GPS_LOCATION_HAS_ALTITUDE;
        if (getData(DATA_LASTGOOD_ERROR_RADIUS_METERS, loc.accuracy))
          loc.flags |= GPS_LOCATION_HAS_ACCURACY;

        // speed / heading
        if (getData(DATA_TRUE_HEADING_DEGREES, loc.bearing))
          loc.flags |= GPS_LOCATION_HAS_BEARING;
        if (getData(DATA_SPEED_KNOTS, loc.speed))
        {
          loc.speed *= (1.852 / 3.6); // 1 knots -> 1.852km / hr -> 1.852 * 1000 / 3600 m/s
          loc.flags |= GPS_LOCATION_HAS_SPEED;
        }
      }

#if (PLATFORM_SDK_VERSION >= 26 /* >=8.0 */)
      GpsLocationExt loc_ext{};

      if (getData(DATA_UBX_ACCURACY_VERTICAL_M, loc_ext.vertical_accuracy_meters))
        loc.flags |= GPS_LOCATION_HAS_VERTICAL_ACCURACY;

      if (getData(DATA_UBX_ACCURACY_SPEED_MPS, loc_ext.speed_accuracy_meters_per_second))
        loc.flags |= GPS_LOCATION_HAS_SPEED_ACCURACY;

      if (getData(DATA_UBX_ACCURACY_HEADING_DEG, loc_ext.bearing_accuracy_degrees))
        loc.flags |= GPS_LOCATION_HAS_BEARING_ACCURACY;
#endif /* >=8.0 */

      TIMESTAMP ts{};
      double ttff = -1;
      if ((loc.flags & GPS_LOCATION_HAS_LAT_LONG) && getData(DATA_UBX_TTFF, ttff) &&
          getData(DATA_UTC_TIMESTAMP, ts))
      {
        LOGGPS(0x00000002,
               "%04d%02d%02d%02d%02d%06.3f,%10.6f,%11.6f,%d "
               "#position(time_stamp,lat,lon,ttff)",
               ts.wYear,
               ts.wMonth,
               ts.wDay,
               ts.wHour,
               ts.wMinute,
               1e-6 * ts.lMicroseconds,
               loc.latitude,
               loc.longitude,
               (int)((1e-3 * ttff) + 0.5));

#ifdef SUPL_ENABLED
        logAgps.write(0x00000003,
                      "%d, "
                      "%04d%02d%02d%02d%02d%06.3f,%10.6f,%11.6f,%d "
                      "#position(time_stamp,lat,lon,ttff)",
                      0,
                      ts.wYear,
                      ts.wMonth,
                      ts.wDay,
                      ts.wHour,
                      ts.wMinute,
                      1e-6 * ts.lMicroseconds,
                      loc.latitude,
                      loc.longitude,
                      (int)((1e-3 * ttff) + 0.5));
#endif
      }
      loc.timestamp = GetGpsUtcTime();
      if ((loc.flags != 0) && (m_publishCount > 0))
      {
#if (PLATFORM_SDK_VERSION >= 26 /* >=8.0 */)
        loc_ext.gps_location = loc;
        CGpsIf::getInstance()->m_callbacks.location_cb(&loc_ext.gps_location);
#else
        CGpsIf::getInstance()->m_callbacks.location_cb(&loc);
#endif /* >=8.0 */
      }
    }

#if (PLATFORM_SDK_VERSION >= 24 /* >=7.0 */)
    if (CGpsIf::getInstance()->m_callbacks.gnss_sv_status_cb)
#else
    if (CGpsIf::getInstance()->m_callbacks.sv_status_cb)
#endif
    {
      CommitSvStatus();
    }
  }
  return state;
}

bool CAndroidDatabase::GetCurrentTimestamp(TIMESTAMP &ft)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  time_t tt = tv.tv_sec;
  long usec = tv.tv_usec;

  struct tm st;
  gmtime_r(&tt, &st);
  ft.wYear = (U2)(st.tm_year + 1900);
  ft.wMonth = (U2)(st.tm_mon + 1);
  ft.wDay = (U2)(st.tm_mday);
  ft.wHour = (U2)(st.tm_hour);
  ft.wMinute = (U2)(st.tm_min);
  ft.lMicroseconds = (unsigned long)st.tm_sec * 1000000 + (unsigned long)usec;

  return true;
}

void CAndroidDatabase::setEpochInterval(int timeIntervalMs, int64_t nextReportEpochMs)
{
  std::lock_guard<std::mutex> guard(m_timeIntervalMutex);

  m_timeInterval = timeIntervalMs;
  m_nextReportEpochMs = nextReportEpochMs;
}

int CAndroidDatabase::getAgc()
{
  U2 agc;
  if (!getData(DATA_UBX_AGC, agc))
  {
    UBX_LOG(LCAT_ERROR, "failed to get AGC");
    return -1;
  }

  return agc;
}

void CAndroidDatabase::incPublish(void)
{
  assert(m_publishCount >= 0);
  m_publishCount++;
  UBX_LOG(LCAT_VERBOSE, "count now %i", m_publishCount);
}

void CAndroidDatabase::decPublish(void)
{
  if (m_publishCount > 0)
  {
    m_publishCount--;
  }
  UBX_LOG(LCAT_VERBOSE, "count now %i", m_publishCount);
}

void CAndroidDatabase::LockOutputDatabase(void) {}

void CAndroidDatabase::UnlockOutputDatabase(void) {}
