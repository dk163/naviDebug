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
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_localDb.h $
 *****************************************************************************/

#ifndef __UBX_LOCALDB_H__
#define __UBX_LOCALDB_H__

#include "database.h"
#include "gps_thread.h"

///////////////////////////////////////////////////////////////////////////////

class CAndroidDatabase : public CDatabase
{
protected:
    ControlThreadInfo* m_pGpsState;
    //int64_t m_lastReportTime; // Debug
	
	// time interval delay
	pthread_mutex_t         m_timeIntervalMutex;
	int                     m_timeInterval;
	int64_t                 m_nextReportEpochMs;
	int 					m_publishCount;

	virtual bool GetCurrentTimestamp(TIMESTAMP& ft);
	
public:
    CAndroidDatabase();
	~CAndroidDatabase();

	static CAndroidDatabase* getInstance();
    GpsUtcTime GetGpsUtcTime(void) const;

	virtual STATE_t Commit(bool bClear);

	void setEpochInterval(int timeIntervalMs, int64_t nextReportEpochMs);
	void setGpsState(ControlThreadInfo* pGpsState) { m_pGpsState = pGpsState; };
	int64_t getNextReportEpoch(void) const { return m_nextReportEpochMs; };
	void incPublish(void);
	void decPublish(void);
	void resetPublish(void) { m_publishCount = 0; };
	

	template<typename T> bool getData(DATA_t data, T &v) const
	{
		if (data < DATA_NUM)
		{
			return m_varO[data].Get(v);
		}
		return false;
	}
protected:
	virtual void LockOutputDatabase(void);
	virtual void UnlockOutputDatabase(void);

};



#endif /* __UBX_LOCALDB_H__ */
