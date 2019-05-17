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
/*!
  \file
  \brief Contains the definitions for logging to file or Android log (logcat)

  Besides the definition of the \ref CLog class, this file also contains
  definitions for various helper functions/macros and everything required to
  log to the standard Android logging mechanism which can be viewed with logcat.
*/

#pragma once
#include <map>
#include <vector>

#ifdef __GNUC__
// make asprintf and vasprintf available and
// use them with ubx_ prefix
#define ubx_asprintf asprintf
#define ubx_vasprintf vasprintf
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif // ifndef _GNU_SOURCE
#endif // ifdef __GNUC__

#if !defined(_GNU_SOURCE) || !defined(__GNUC__)
//! Allocates size bytes of memory and writes the string to it like vsnprintf
int ubx_vasnprintf(char **buf, size_t size, char const *format, va_list va);

//! Allocates size bytes of memory and writes the string to it like snprintf
int ubx_asnprintf(char **buf, size_t size, char const *format, ...);

//! Writes characters like vasprintf to newly allocated memory
int ubx_vasprintf(char **buf, char const *format, va_list va);
//
//! Writes characters like asprintf to newly allocated memory
int ubx_asprintf(char **buf, char const *format, ...);
#endif

#ifndef LOG_TAG
#define LOG_TAG "u-blox" //!< Tag for logging
#endif
#ifndef LOG_NDEBUG
#define LOG_NDEBUG 0
#endif
#include <cutils/log.h>
#include <string>

#ifdef SUPL_ENABLED
#include "PDU.h"
#include "ULP-PDU.h"
#endif

// macro names have changed since Android 4.0.3 (ALOGV() instead of
// LOGV(LCAT_VERBOSE, ) etc. in 4.0.9)
// see system/core/include/cutils/log.h
#ifndef LOG
#define LOG ALOG
#endif
#ifndef LOGV
#define LOGV ALOGV
#endif
#ifndef LOGD
#define LOGD ALOGD
#endif
#ifndef LOGI
#define LOGI ALOGI
#endif
#ifndef LOGW
#define LOGW ALOGW
#endif
#ifndef LOGE
#define LOGE ALOGE
#endif

#define LOG_SPECIAL_BUFFER_COUNT 4
#if defined MALLOC_DEBUG

#define MC_MALLOC(size) debug_malloc(size)
#define MC_CALLOC(size, num) debug_calloc(size, num)
#define MC_FREE(data) debug_free(data)

inline void *debug_malloc(size_t size);
inline void *debug_calloc(size_t size, size_t num);
inline void debug_free(void *data);

#else

#define MC_MALLOC(size) malloc(size)           //!< macro for debug malloc
#define MC_CALLOC(size, num) calloc(size, num) //!< macro for debug calloc
#define MC_FREE(data) free(data)               //!< macro for debugging free

#endif

///////////////////////////////////////////////////////////////////////////////
// function and class name printing

// _func_ is part of C99, but some older GCC might only support __FUNCTION__
#ifndef __func__
#if __STDC_VERSION__ < 199901L // No C99 compiler
#if __GNUC__ >= 2              // GCC supports __FUNCTION__ instead
#define __func__ __FUNCTION__
#else // GCC does not even support __FUNCTION__
#define __func__ ""
#endif
#endif
#endif

// We only want to parse __PRETTY_FUNCTION__ in g++
#ifdef __GNUG__
#define __CPP_FUNC_SIGNATURE__ __PRETTY_FUNCTION__
#else
#define __CPP_FUNC_SIGNATURE__ ""
#endif

//! Macro for logging to default Android log (logcat)
/*! This Macro acts as a wrapper for \ref _ubx_log and makes it possible to log
        the current text in fprintf style to logcat. When logging, the function
        prepends the function name (including class - if possible). If NDEBUG is
        not defined, the file name and the line number from which the this macro
   is
        called will be prepended as well.
                \param pri : The priority as defined in \ref LCAT_LOGLEVEL of
   the
                                         messsage
                \param fmt : The formatting of the string
                \param ... : The parameters formatted by fmt
*/
#define UBX_LOG(pri, fmt, ...)                                                                     \
  _ubx_log(pri, __FILE__, (long int)__LINE__, __func__, __CPP_FUNC_SIGNATURE__, fmt, ##__VA_ARGS__)

typedef enum
{
  LCAT_ERROR,
  LCAT_WARNING,
  LCAT_INFO,
  LCAT_VERBOSE,
  LCAT_DEBUG
} LCAT_LOGLEVEL;

int _ubx_log(LCAT_LOGLEVEL pri,
             char const *const file,
             long int const line,
             char const *const c_func,
             char const *const cpp_func,
             char const *const fmt,
             ...);
///////////////////////////////////////////////////////////////////////////////
// value to string conversion helpers
namespace ublox
{
  namespace log
  {
    namespace stringtables
    {
      extern std::vector<std::string> strGpsPositionMode;
      extern std::vector<std::string> strGpsPositionRecurrence;
      extern std::vector<std::string> strGpsStatusValue;
      extern std::vector<std::string> xstrGpsLocationFlags;
      extern std::vector<std::string> xstrGpsCapabilityFlags;
      extern std::vector<std::string> xstrGpsAidingData;
      extern std::vector<std::string> strAGpsType;
      extern std::vector<std::string> strAGpsSetIDType;
      extern std::vector<std::string> strGpsNiType;
      extern std::vector<std::string> xstrGpsNiNotifyFlags;
      extern std::vector<std::string> strGpsUserResponseType;
      extern std::vector<std::string> strGpsNiEncodingType;
      extern std::vector<std::string> strAGpsStatusValue;
      extern std::vector<std::string> strAGpsRefLocation;
      extern std::vector<std::string> strAgpsRilNetworkType;
      extern std::vector<std::string> xstrAgpsRilRequestSetId;
      extern std::vector<std::string> xstrAgpsRilRequestRefLoc;
    } // namespace stringtables
  }   // namespace log
} // namespace ublox
#ifndef _LOOKUPSTR
#define _LOOKUPSTR(v, t) _strLookup(v, str##t)
#endif
#ifndef _LOOKUPSTRX
#define _LOOKUPSTRX(v, t) _strLookupX(v, ublox::log::stringtables::xstr##t)
#endif

const char *_strLookup(unsigned int v, const std::vector<std::string> &vec);
const char *_strLookup(unsigned int v, const std::map<unsigned int, std::string> &map);
const char *_strLookupX(unsigned int v, const std::vector<std::string> &vec);

#ifdef SUPL_ENABLED
///////////////////////////////////////////////////////////////////////////////

static const unsigned int NAME_SIZE = 256;

class CLog
{
public:
  CLog(const char *name = "GPS.LOG", int max = 64 * 1024, bool verbose = false);
  ~CLog();

  void write(unsigned int code, const char *fmt, ...);
  void txt(int code, const char *pTxt);
  void writeFile(const char *pBuf, int len);

protected:
  static const size_t BUF_SIZE;
  static const size_t TIMESTRING_SIZE;
  void open(const char *name, int max);
  static const char *timestamp(char *buf = NULL);
  char m_name[NAME_SIZE];
  int m_max;
  bool m_verbose;
};

void logSupl(const struct ULP_PDU *pMsg, bool incoming);
void logRRLP(const PDU_t *pRrlpMsg, bool incoming);

extern CLog logGps;
extern CLog logAgps;
#define LOGGPS(code, ...) logGps.write(code, __VA_ARGS__)
#define LOGAGPS(code, ...) logApps.write(code, __VA_ARGS__)
#else
#define LOGGPS(code, ...)
#define LOGAGPS(code, ...)
#endif
std::string byte_array_to_hex_string(unsigned char const *pData, int dataLen);
