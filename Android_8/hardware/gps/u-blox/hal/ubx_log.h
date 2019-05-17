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
 * $Id: ubx_log.h 104014 2015-10-10 09:45:41Z marcus.picasso $
 * $HeadURL: http://svn.u-blox.ch/GPS/SOFTWARE/hmw/android/release/v3.30/gps/hal/ubx_log.h $
 *****************************************************************************/
/*!
  \file
  \brief Contains the definitions for logging to file or Android log (logcat)

  Besides the definition of the \ref CLog class, this file also contains
  definitions for various helper functions/macros and everything required to 
  log to the standard Android logging mechanism which can be viewed with logcat.
*/

#ifndef __UBX_LOG_H__
#define __UBX_LOG_H__

#ifdef __GNUC__
// make asprintf and vasprintf available and
// use them with ubx_ prefix
# define ubx_asprintf asprintf
# define ubx_vasprintf vasprintf
# ifndef _GNU_SOURCE
#  define _GNU_SOURCE
# endif //ifndef _GNU_SOURCE
#endif //ifdef __GNUC__

#if !defined(_GNU_SOURCE) || !defined(__GNUC__)
//! Allocates size bytes of memory and writes the string to it like vsnprintf
int ubx_vasnprintf(char ** buf, size_t size, char const * format, va_list va);

//! Allocates size bytes of memory and writes the string to it like snprintf
int ubx_asnprintf(char ** buf, size_t size, char const * format, ...);

//! Writes characters like vasprintf to newly allocated memory
int ubx_vasprintf(char ** buf, char const * format, va_list va);
    //
//! Writes characters like asprintf to newly allocated memory
int ubx_asprintf(char ** buf, char const * format, ...);
#endif

#define LOG_TAG "u-blox"        //!< Tag for logging
#define LOG_NDEBUG 0
#include <cutils/log.h>

#ifdef SUPL_ENABLED
#include "ULP-PDU.h"
#include "PDU.h"
#endif

// macro names have changed since Android 4.0.3 (ALOGV() instead of LOGV(LCAT_VERBOSE, ) etc. in 4.0.9)
// see system/core/include/cutils/log.h
#ifndef LOG
#  define LOG ALOG
#endif
#ifndef LOGV
#  define LOGV ALOGV
#endif
#ifndef LOGD
#  define LOGD ALOGD
#endif
#ifndef LOGI
#  define LOGI ALOGI
#endif
#ifndef LOGW
#  define LOGW ALOGW
#endif
#ifndef LOGE
#  define LOGE ALOGE
#endif




#define LOG_SPECIAL_BUFFER_COUNT 4
#if defined MALLOC_DEBUG

#define MC_MALLOC(size) debug_malloc(size)
#define MC_CALLOC(size,num) debug_calloc(size,num)
#define MC_FREE(data) debug_free(data)

inline void *debug_malloc(size_t size);
inline void *debug_calloc(size_t size, size_t num);
inline void  debug_free(void *data);

#else

#define MC_MALLOC(size) malloc(size)            //!< macro for debug malloc
#define MC_CALLOC(size,num) calloc(size,num)    //!< macro for debug calloc
#define MC_FREE(data) free(data)                //!< macro for debugging free

#endif

///////////////////////////////////////////////////////////////////////////////
// function and class name printing

// _func_ is part of C99, but some older GCC might only support __FUNCTION__
#ifndef __func__
# if __STDC_VERSION__ < 199901L // No C99 compiler
#  if __GNUC__ >= 2 // GCC supports __FUNCTION__ instead
#   define __func__ __FUNCTION__
#  else // GCC does not even support __FUNCTION__
#   define __func__ ""
#  endif
# endif
#endif

// We only want to parse __PRETTY_FUNCTION__ in g++
#ifdef __GNUG__
# define __CPP_FUNC_SIGNATURE__ __PRETTY_FUNCTION__
#else
# define __CPP_FUNC_SIGNATURE__ ""
#endif
 
//! Macro for logging to default Android log (logcat)
/*! This Macro acts as a wrapper for \ref _ubx_log and makes it possible to log
	the current text in fprintf style to logcat. When logging, the function 
	prepends the function name (including class - if possible). If NDEBUG is
	not defined, the file name and the line number from which the this macro is
   	called will be prepended as well.
		\param pri : The priority as defined in \ref LCAT_LOGLEVEL of the 
					 messsage
		\param fmt : The formatting of the string
		\param ... : The parameters formatted by fmt
*/
#define UBX_LOG(pri, fmt, ...) \
    _ubx_log(  pri \
		    , __FILE__\
		    , (long int)__LINE__\
		    , __func__\
		    , __CPP_FUNC_SIGNATURE__\
			, fmt\
		    , ## __VA_ARGS__)

typedef enum 
{
	LCAT_ERROR,
	LCAT_WARNING,
	LCAT_INFO,
	LCAT_VERBOSE,	
	LCAT_DEBUG
} LCAT_LOGLEVEL;

int _ubx_log(LCAT_LOGLEVEL pri
			, char const * const file
			, long int const line
			, char const * const c_func
			, char const * const cpp_func
			, char const * const fmt
			, ...);
///////////////////////////////////////////////////////////////////////////////
// value to string conversion helpers

static const char* _strGpsPositionMode[] = {
	"GPS_POSITION_MODE_STANDALONE",
	"GPS_POSITION_MODE_MS_BASED",
	"GPS_POSITION_MODE_MS_ASSISTED"
};

static const char*_strGpsPositionRecurrence[] = {
	"GPS_POSITION_RECURRENCE_PERIODIC",
	"GPS_POSITION_RECURRENCE_SINGLE"
};

static const char* _strGpsStatusValue[] = {
	"GPS_STATUS_NONE",
	"GPS_STATUS_SESSION_BEGIN",
	"GPS_STATUS_SESSION_END",
	"GPS_STATUS_ENGINE_ON",
	"GPS_STATUS_ENGINE_OFF",
};

static const char* _xstrGpsLocationFlags[] = {
    "GPS_LOCATION_HAS_LAT_LONG",
    "GPS_LOCATION_HAS_ALTITUDE",
    "GPS_LOCATION_HAS_SPEED",
    "GPS_LOCATION_HAS_BEARING",
    "GPS_LOCATION_HAS_ACCURACY",
};

static const char* _xstrGpsCapabilityFlags[] = {
    "GPS_CAPABILITY_SCHEDULING",
    "GPS_CAPABILITY_MSB",
    "GPS_CAPABILITY_MSA",
    "GPS_CAPABILITY_SINGLE_SHOT",
    "GPS_CAPABILITY_ON_DEMAND_TIME",
};

static const char* _xstrGpsAidingData[] = {
    "GPS_DELETE_EPHEMERIS",
    "GPS_DELETE_ALMANAC",
    "GPS_DELETE_POSITION",
    "GPS_DELETE_TIME",
    "GPS_DELETE_IONO",
    "GPS_DELETE_UTC",
    "GPS_DELETE_HEALTH",
    "GPS_DELETE_SVDIR",
    "GPS_DELETE_SVSTEER",
    "GPS_DELETE_SADATA",
    "GPS_DELETE_RTI",
	NULL,
	NULL,
	NULL,
	NULL,
    "GPS_DELETE_CELLDB_INFO",
    // "GPS_DELETE_ALL", 0xFFFF
};

static const char* _strAGpsType[] = {
	NULL,
	"AGPS_TYPE_SUPL",
	"AGPS_TYPE_C2K",
};

static const char* _strAGpsSetIDType[] = {
	"AGPS_SETID_TYPE_NONE",
	"AGPS_SETID_TYPE_IMSI",
    "AGPS_SETID_TYPE_MSISDN",
};

static const char* _strGpsNiType[] = {
	NULL,
    "GPS_NI_TYPE_VOICE",
    "GPS_NI_TYPE_UMTS_SUPL",
    "GPS_NI_TYPE_UMTS_CTRL_PLANE",
};

static const char* _xstrGpsNiNotifyFlags[] = {
    "GPS_NI_NEED_NOTIFY",
    "GPS_NI_NEED_VERIFY",
    "GPS_NI_PRIVACY_OVERRIDE",
};

static const char* _strGpsUserResponseType[] = {
	NULL,
    "GPS_NI_RESPONSE_ACCEPT",
    "GPS_NI_RESPONSE_DENY",
    "GPS_NI_RESPONSE_NORESP",
};

static const char* _strGpsNiEncodingType[] = {
    // "GPS_ENC_UNKNOWN", = -1
    "GPS_ENC_NONE",
    "GPS_ENC_SUPL_GSM_DEFAULT",
    "GPS_ENC_SUPL_UTF8",
    "GPS_ENC_SUPL_UCS2",
};

static const char* _strAGpsStatusValue[] = {
	NULL,
	"GPS_REQUEST_AGPS_DATA_CONN",
	"GPS_RELEASE_AGPS_DATA_CONN",
	"GPS_AGPS_DATA_CONNECTED",
	"GPS_AGPS_DATA_CONN_DONE",
	"GPS_AGPS_DATA_CONN_FAILED",
};

static const char* _strAGpsRefLocation[] = {
	NULL,
	"AGPS_REF_LOCATION_TYPE_GSM_CELLID",
	"AGPS_REF_LOCATION_TYPE_UMTS_CELLID",
	"AGPS_REF_LOCATION_TYPE_TYPE_MAC",
};

static const char* _strAgpsRilNetworkType[] = { // Network types for update_network_state "type" parameter
    "AGPS_RIL_NETWORK_TYPE_MOBILE",
    "AGPS_RIL_NETWORK_TYPE_WIFI",
    "AGPS_RIL_NETWORK_TYPE_MOBILE_MMS",
    "AGPS_RIL_NETWORK_TYPE_MOBILE_SUPL",
    "AGPS_RIL_NETWORK_TTYPE_MOBILE_DUN",
    "AGPS_RIL_NETWORK_TTYPE_MOBILE_HIPRI",
    "AGPS_RIL_NETWORK_TTYPE_WIMAX",
};

static const char* _xstrAgpsRilRequestSetId[] = {
	"AGPS_RIL_REQUEST_SETID_IMSI",
	"AGPS_RIL_REQUEST_SETID_MSISDN",
};

static const char* _xstrAgpsRilRequestRefLoc[] = {
	"AGPS_RIL_REQUEST_REFLOC_CELLID",
	"AGPS_RIL_REQUEST_REFLOC_MAC",
};

#ifndef _LOOKUPSTR
# define _LOOKUPSTR(v, t) _strLookup(v, _str##t, sizeof(_str##t)/sizeof(_str##t[0]) )
#endif
#ifndef _LOOKUPSTRX
# define _LOOKUPSTRX(v, t) _strLookupX(v, _xstr##t, sizeof(_xstr##t)/sizeof(_xstr##t[0]) )
#endif

const char* _strLookup(unsigned int v, const char* const * l, unsigned int n);
const char* _strLookupX(unsigned int v, const char* const * l, unsigned int n);

#ifdef SUPL_ENABLED
///////////////////////////////////////////////////////////////////////////////

class CLog
{
public:
	CLog(const char* name = "GPS.LOG", int max=64*1024, bool verbose = false);
	~CLog();
	
	void write(unsigned int code, const char* fmt, ...);
	void txt(int code, const char* pTxt);
	void writeFile(const char* pBuf, int len);

protected:
	static const size_t BUF_SIZE;
	static const size_t TIMESTRING_SIZE;
	void open(const char* name, int max);
	static const char* timestamp(char* buf = NULL);
	char m_name[256];
	int m_max;
	bool m_verbose;
};

void logSupl(const struct ULP_PDU * pMsg, bool incoming);
void logRRLP(const PDU_t *pRrlpMsg, bool incoming);

extern CLog	logGps;
extern CLog logAgps;
#define LOGGPS(code, ...)  logGps.write(code, __VA_ARGS__)
#define LOGAGPS(code, ...) logApps.write(code, __VA_ARGS__)
#else
#define LOGGPS(code, ...)
#define LOGAGPS(code, ...)
#endif
int byte_array_to_hex_string(char **pResult, unsigned char const * pData, int dataLen);

#endif /* __UBX_LOG_H__ */
