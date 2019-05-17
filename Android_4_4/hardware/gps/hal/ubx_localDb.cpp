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
 * $Id: ubx_localDb.cpp 108175 2015-12-14 13:19:24Z fabio.robbiani $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_localDb.cpp $
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <sys/time.h>

#include "ubx_log.h"
#include "std_types.h"
#include "std_lang_def.h"
#include "std_macros.h"
#include "ubx_timer.h"

#include "ubx_localDb.h"

static CAndroidDatabase s_database;

CAndroidDatabase::CAndroidDatabase()
	: CDatabase()
{
	m_pGpsState = NULL;
	
	pthread_mutex_init(&m_timeIntervalMutex, NULL);
	m_timeInterval = 0;
	m_nextReportEpochMs = 0;
	// m_lastReportTime = time(NULL) * 1000; // Debug
	m_publishCount = 0;					// Publishing off by default;
}

CAndroidDatabase::~CAndroidDatabase()
{
	pthread_mutex_destroy(&m_timeIntervalMutex);
	m_pGpsState = NULL;
}

CAndroidDatabase* CAndroidDatabase::getInstance()
{
	return &s_database;
}

GpsUtcTime CAndroidDatabase::GetGpsUtcTime(void) const
{
    TIMESTAMP ts;
    if (getData(DATA_UTC_TIMESTAMP, ts))
    {
        struct tm ti;
        memset(&ti,0,sizeof(ti));
        ti.tm_year  = ts.wYear - 1900;
        ti.tm_mon   = ts.wMonth - 1;
        ti.tm_mday  = ts.wDay;
        ti.tm_hour  = ts.wHour;
        ti.tm_min   = ts.wMinute;
        ti.tm_sec   = (int) (ts.lMicroseconds / 1000000);
        unsigned long us = (unsigned long) (ts.lMicroseconds - (unsigned long) ti.tm_sec * 1000000);
        ti.tm_isdst = -1;

        time_t t = mktime(&ti);
//            UBX_LOG(LCAT_VERBOSE, "%s", ctime(&t));

        // calc utc / local difference
        time_t         now = time(NULL);
        struct tm      tmLocal;
        struct tm      tmUtc;
        long           timeLocal, timeUtc;

// Hi gaode map, let us fuck you!
#if 0
        gmtime_r( &now, &tmUtc );
        localtime_r( &now, &tmLocal );
        timeLocal =	tmLocal.tm_sec +
            60*(tmLocal.tm_min +
                60*(tmLocal.tm_hour +
                    24*(tmLocal.tm_yday +
                        365*tmLocal.tm_year)));
        timeUtc =	tmUtc.tm_sec +
            60*(tmUtc.tm_min +
                60*(tmUtc.tm_hour +
                    24*(tmUtc.tm_yday +
                        365*tmUtc.tm_year)));
        long utcDiff = timeUtc - timeLocal;
//            UBX_LOG(LCAT_VERBOSE, "Time utcDiff: %li", utcDiff);

        t -= utcDiff;
//            UBX_LOG(LCAT_VERBOSE, "%s", ctime(&t));
#endif

        GpsUtcTime gpsUtcTime = (GpsUtcTime) t  * (GpsUtcTime) 1000 + (GpsUtcTime) (us / 1000);
		

        return gpsUtcTime;
    }
    return 0;
}

CDatabase::STATE_t CAndroidDatabase::Commit(bool bClear)
{
    CDatabase::STATE_t state;

    // Store commit time in database
    state = CDatabase::Commit(bClear);

    //UBX_LOG(LCAT_VERBOSE, "Perform commit: clear %i   state %i", bClear, state);

    //UBX_LOG(LCAT_VERBOSE, "*** Epoch *** Now %lli, Last %lli, Interval %lli",
    //    now, lastReportTime, reportInterval);

    // lastReportTime = time(NULL) * 1000; // Debug

    pthread_mutex_lock(&m_timeIntervalMutex);
    int report = false;
    //UBX_LOG(LCAT_VERBOSE, "Ready to report");
    if (m_nextReportEpochMs)
    {
        int64_t monotonicNow = getMonotonicMsCounter();
        if (monotonicNow >= m_nextReportEpochMs)
        {
            report = true;  // Yes we can report
            m_nextReportEpochMs = monotonicNow + m_timeInterval;
            //UBX_LOG(LCAT_VERBOSE, "Next timer epoch (%lli)", m_nextReportEpochMs);
        }
    }
    else
    {
        report = true;  // Yes we can report
        //UBX_LOG(LCAT_VERBOSE, "Normal report epoch");
    }
    pthread_mutex_unlock(&m_timeIntervalMutex);

	if ((m_pGpsState != NULL) && (m_pGpsState->gpsState == GPS_STARTED) && (report))
    {
        // Driver in the 'started' state, so ok to report to framework
        //UBX_LOG(LCAT_VERBOSE, ">>> Reporting (%p)", m_pGpsState->pGpsInterface);
        // Location
        if (CGpsIf::getInstance()->m_callbacks.location_cb)
        {
            GpsLocation loc;
            memset(&loc, 0, sizeof(GpsLocation));
            IF_ANDROID23( loc.size = sizeof(GpsLocation); )
            // position
			if (state == STATE_READY)
			{
				//UBX_LOG(LCAT_VERBOSE, ">>> Reporting Pos, Alt & Acc");
				if (getData(DATA_LASTGOOD_LATITUDE_DEGREES, loc.latitude) && getData(DATA_LASTGOOD_LONGITUDE_DEGREES, loc.longitude))
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
					loc.speed *=  (1.852 / 3.6); // 1 knots -> 1.852km / hr -> 1.852 * 1000 / 3600 m/s
					loc.flags |= GPS_LOCATION_HAS_SPEED;
				}
			}
			
            TIMESTAMP ts;
			memset(&ts, 0, sizeof(ts));	
            double ttff=-1;
            if ((loc.flags & GPS_LOCATION_HAS_LAT_LONG)
                && getData(DATA_UBX_TTFF, ttff)
                && getData(DATA_UTC_TIMESTAMP, ts))
            {
                LOGGPS(0x00000002, "%04d%02d%02d%02d%02d%06.3f,%10.6f,%11.6f,%d #position(time_stamp,lat,lon,ttff)",
                       ts.wYear, ts.wMonth, ts.wDay, ts.wHour, ts.wMinute, 1e-6*ts.lMicroseconds,
                       loc.latitude, loc.longitude, (int) ((1e-3 * ttff) + 0.5));

#ifdef SUPL_ENABLED
                logAgps.write(0x00000003, "%d, %04d%02d%02d%02d%02d%06.3f,%10.6f,%11.6f,%d #position(time_stamp,lat,lon,ttff)",
                              0, ts.wYear, ts.wMonth, ts.wDay, ts.wHour, ts.wMinute, 1e-6*ts.lMicroseconds, 
                              loc.latitude, loc.longitude, (int) ((1e-3 * ttff) + 0.5));
#endif
            }
			loc.timestamp = GetGpsUtcTime();
			if ((loc.flags != 0) && (m_publishCount > 0))
            {
             	CGpsIf::getInstance()->m_callbacks.location_cb(&loc);
            }
        }

        if (CGpsIf::getInstance()->m_callbacks.sv_status_cb)
        {
            int i;
            // Satellite status
            GpsSvStatus svStatus;
            memset(&svStatus, 0, sizeof(GpsSvStatus));
            IF_ANDROID23( svStatus.size = sizeof(GpsSvStatus); )

            // we do publish the satellites to the android framework the qualcomm way (like done in nmea)
#define PRN_MASK(prn) (1ul << (prn - 1)) // convert to the prn mask

            // Satellites in view
            int numSvInDb = 0;
            if (getData(DATA_SATELLITES_IN_VIEW, numSvInDb) && (numSvInDb > 0))
            {
                int numValidSv = 0;
                for (i = 0; (i < numSvInDb) && (numValidSv < GPS_MAX_SVS); i++)
                {
                    float az = 0.0f, el = 0.0f, snr = 0.0f;
                    int prn = 0;
                    int gnssId = GNSSID_UNKN;
                    bool azelOk = (getData(DATA_SATELLITES_IN_VIEW_ELEVATION_(i), el)
                               && (el >= 0.0f) && (el <= 90.0f))
                               && (getData(DATA_SATELLITES_IN_VIEW_AZIMUTH_(i), az)
                               && (az >= 0.0f) && (az <= 360.0f));
                    bool snrOk  = (getData(DATA_SATELLITES_IN_VIEW_STN_RATIO_(i), snr) && (snr > 0.0f));
                    getData(DATA_SATELLITES_IN_VIEW_SVID_(i), prn);
                    getData(DATA_SATELLITES_IN_VIEW_GNSSID_(i), gnssId);

                    if (  azelOk /* elevation and azimuth exist and are fine */
                      ||  snrOk  /* The signal to noise ratio exists and is fine */
                      || (prn && gnssId >= GNSSID_GPS && gnssId <= GNSSID_GLO) /* The prn is known for this satellite and type is valid */ )
                    {
                        int navsta;
						IF_ANDROID23( svStatus.sv_list[numValidSv].size = sizeof(GpsSvInfo); )
                        int prnOffset = 0;

                        // The Galileo and GPS SV IDs overlap, map Galileo to 101-136
                        if ( gnssId == GNSSID_GAL
                          && prn    >=  1
                          && prn    <= 36 )
                        {
                            prnOffset = 100;
                        }

                        // The BeiDou and GPS SV IDs overlap, map BeiDou to 201-237
                        if ( gnssId == GNSSID_BDS
                          && prn    >=  1
                          && prn    <= 37 )
                        {
                            prnOffset = 200;
                        }

                        svStatus.sv_list[numValidSv].prn       = prn + prnOffset;
                        svStatus.sv_list[numValidSv].azimuth   = azelOk ? az : -1;
                        svStatus.sv_list[numValidSv].elevation = azelOk ? el : -1;
                        svStatus.sv_list[numValidSv].snr       = snrOk ? snr : -1;


                        // For GPS SVs additional information can be provided to the framework
                        if (getData(DATA_UBX_SATELLITES_IN_VIEW_NAV_STA_(i), navsta))
                        {
                            // Currently it is only possible to report availability
                            // of ALM or EPH for GPS 
                            if(gnssId == GNSSID_GPS)
                            {
                                if (navsta & IS_USED)
                                {
                                    svStatus.used_in_fix_mask |= PRN_MASK(prn);
                                }

                                if (navsta & HAS_EPH)
                                {
                                    svStatus.ephemeris_mask   |= PRN_MASK(prn);
                                }

                                if (navsta & HAS_ALM)
                                {
                                    svStatus.almanac_mask     |= PRN_MASK(prn);
                                }
                            }
#ifdef UBX_SV_STATUS_LOGGING
                            UBX_LOG(LCAT_VERBOSE, "SV: %i:%i -> %i status: %c%c%c", gnssId, prn, prn+prnOffset, navsta & IS_USED?'U':'u',  navsta & HAS_EPH?'E':'e', navsta & HAS_ALM?'A':'a' );
#endif //UBX_SV_STATUS_LOGGING
                        }

                        ++numValidSv;
                    }
                }

                // Report SV data to the framework
                if(numValidSv)
                {
                    svStatus.num_svs = numValidSv;
                    CGpsIf::getInstance()->m_callbacks.sv_status_cb(&svStatus);
#ifdef UBX_SV_STATUS_LOGGING
                    UBX_LOG(LCAT_VERBOSE, "SV status updated!");
#endif //UBX_SV_STATUS_LOGGING
                }
            }
        }
    }
    return state;
}


bool CAndroidDatabase::GetCurrentTimestamp(TIMESTAMP& ft)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_t tt = tv.tv_sec;
    long usec = tv.tv_usec;

	struct tm st;
    gmtime_r(&tt, &st);
    ft.wYear		 = (U2) (st.tm_year + 1900);	
    ft.wMonth		 = (U2) (st.tm_mon + 1);
    ft.wDay		 	 = (U2) (st.tm_mday);
    ft.wHour		 = (U2) (st.tm_hour);
    ft.wMinute		 = (U2) (st.tm_min);
    ft.lMicroseconds = (unsigned long) st.tm_sec * 1000000 + (unsigned long) usec;

    return true;
}

void CAndroidDatabase::setEpochInterval(int timeIntervalMs, int64_t nextReportEpochMs)
{
	pthread_mutex_lock(&m_timeIntervalMutex);
	
	m_timeInterval = timeIntervalMs;
	m_nextReportEpochMs = nextReportEpochMs;
	
	pthread_mutex_unlock(&m_timeIntervalMutex);
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

void CAndroidDatabase::LockOutputDatabase(void)
{

}

void CAndroidDatabase::UnlockOutputDatabase(void)
{

}

