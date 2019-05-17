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
\brief Contains the implementations for logging to file or Android log (logcat)

Besides the implementation of the \ref CLog functions, this file also contains
various helper functions and everything required to log to the standard
Android logging mechanism which can be viewed with logcat.
*/

#include "private/android_filesystem_config.h"
#include <ctype.h>
#include <cutils/open_memstream.h>
#include <errno.h>
#include <fcntl.h>
#include <grp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "std_types.h"
#include "ubx_log.h"
#include "ubxgpsstate.h"
#include <bitset>
#include <cassert>
#include <iomanip>
#include <sstream>
#include <string>

namespace ublox
{
  namespace log
  {
    namespace stringtables
    {
      namespace
      {
        static const std::string UNKNOWN = "UNKNOWN";
      }
      std::vector<std::string> strGpsPositionMode = {
        "GPS_POSITION_MODE_STANDALONE", //
        "GPS_POSITION_MODE_MS_BASED",   //
        "GPS_POSITION_MODE_MS_ASSISTED" //
      };
      std::vector<std::string> strGpsPositionRecurrence = {
        "GPS_POSITION_RECURRENCE_PERIODIC", //
        "GPS_POSITION_RECURRENCE_SINGLE",   //
      };
      std::vector<std::string> strGpsStatusValue = {
        "GPS_STATUS_NONE",          //
        "GPS_STATUS_SESSION_BEGIN", //
        "GPS_STATUS_SESSION_END",   //
        "GPS_STATUS_ENGINE_ON",     //
        "GPS_STATUS_ENGINE_OFF"     //
      };
      std::vector<std::string> xstrGpsLocationFlags = {
        "GPS_LOCATION_HAS_LAT_LONG", //
        "GPS_LOCATION_HAS_ALTITUDE", //
        "GPS_LOCATION_HAS_SPEED",    //
        "GPS_LOCATION_HAS_BEARING",  //
        "GPS_LOCATION_HAS_ACCURACY", //
      };
      std::vector<std::string> xstrGpsCapabilityFlags = {
        "GPS_CAPABILITY_SCHEDULING",     //
        "GPS_CAPABILITY_MSB",            //
        "GPS_CAPABILITY_MSA",            //
        "GPS_CAPABILITY_SINGLE_SHOT",    //
        "GPS_CAPABILITY_ON_DEMAND_TIME", //
      };
      std::vector<std::string> xstrGpsAidingData = {
        "GPS_DELETE_EPHEMERIS",   //
        "GPS_DELETE_ALMANAC",     //
        "GPS_DELETE_POSITION",    //
        "GPS_DELETE_TIME",        //
        "GPS_DELETE_IONO",        //
        "GPS_DELETE_UTC",         //
        "GPS_DELETE_HEALTH",      //
        "GPS_DELETE_SVDIR",       //
        "GPS_DELETE_SVSTEER",     //
        "GPS_DELETE_SADATA",      //
        "GPS_DELETE_RTI",         //
        UNKNOWN,                  //
        UNKNOWN,                  //
        UNKNOWN,                  //
        UNKNOWN,                  //
        "GPS_DELETE_CELLDB_INFO", //
        // "GPS_DELETE_ALL", 0xFFFF //
      };
      std::vector<std::string> strAGpsType = {
        UNKNOWN,          //
        "AGPS_TYPE_SUPL", //
        "AGPS_TYPE_C2K",  //
      };
      std::vector<std::string> strAGpsSetIDType = {
        "AGPS_SETID_TYPE_NONE",   //
        "AGPS_SETID_TYPE_IMSI",   //
        "AGPS_SETID_TYPE_MSISDN", //
      };
      std::vector<std::string> strGpsNiType = {
        UNKNOWN,                       //
        "GPS_NI_TYPE_VOICE",           //
        "GPS_NI_TYPE_UMTS_SUPL",       //
        "GPS_NI_TYPE_UMTS_CTRL_PLANE", //
      };
      std::vector<std::string> xstrGpsNiNotifyFlags = {
        "GPS_NI_NEED_NOTIFY",      //
        "GPS_NI_NEED_VERIFY",      //
        "GPS_NI_PRIVACY_OVERRIDE", //
      };
      std::vector<std::string> strGpsUserResponseType = {
        UNKNOWN,                  //
        "GPS_NI_RESPONSE_ACCEPT", //
        "GPS_NI_RESPONSE_DENY",   //
        "GPS_NI_RESPONSE_NORESP", //
      };
      std::vector<std::string> strGpsNiEncodingType = {
        // "GPS_ENC_UNKNOWN", = -1 //
        "GPS_ENC_NONE",             //
        "GPS_ENC_SUPL_GSM_DEFAULT", //
        "GPS_ENC_SUPL_UTF8",        //
        "GPS_ENC_SUPL_UCS2",        //
      };
      std::vector<std::string> strAGpsStatusValue = {
        UNKNOWN,                      //
        "GPS_REQUEST_AGPS_DATA_CONN", //
        "GPS_RELEASE_AGPS_DATA_CONN", //
        "GPS_AGPS_DATA_CONNECTED",    //
        "GPS_AGPS_DATA_CONN_DONE",    //
        "GPS_AGPS_DATA_CONN_FAILED",  //
      };
      std::vector<std::string> strAGpsRefLocation = {
        UNKNOWN,                              //
        "AGPS_REF_LOCATION_TYPE_GSM_CELLID",  //
        "AGPS_REF_LOCATION_TYPE_UMTS_CELLID", //
        "AGPS_REF_LOCATION_TYPE_TYPE_MAC",    //
      };
      std::vector<std::string> strAgpsRilNetworkType = {
        "AGPS_RIL_NETWORK_TYPE_MOBILE",        //
        "AGPS_RIL_NETWORK_TYPE_WIFI",          //
        "AGPS_RIL_NETWORK_TYPE_MOBILE_MMS",    //
        "AGPS_RIL_NETWORK_TYPE_MOBILE_SUPL",   //
        "AGPS_RIL_NETWORK_TTYPE_MOBILE_DUN",   //
        "AGPS_RIL_NETWORK_TTYPE_MOBILE_HIPRI", //
        "AGPS_RIL_NETWORK_TTYPE_WIMAX",        //
      };
      std::vector<std::string> xstrAgpsRilRequestSetId = {
        "AGPS_RIL_REQUEST_SETID_IMSI",   //
        "AGPS_RIL_REQUEST_SETID_MSISDN", //
      };
      std::vector<std::string> xstrAgpsRilRequestRefLoc = {
        "AGPS_RIL_REQUEST_REFLOC_CELLID", //
        "AGPS_RIL_REQUEST_REFLOC_MAC",    //
      };
    } // namespace stringtables
  }   // namespace log
} // namespace ublox
#if !defined(_GNU_SOURCE) || !defined(__GNUC__)

// If there is no default vasprintf, we will create it ourselves

/*! ubx_vasnprintf should work like vsnprintf, except that it does allocate
the buffer itself.
\param buf    : Will contain the pointer of the allocated buffer
\param size   : The size of the buffer to be allocated
\param format : The format string
\param va     : The va_list with the arguments
\return         The return value equals the number of written bytes on
\return         success. On error The return value might is -1.
\return         If the call was successful buf must be freed after usage
*/

int ubx_vasnprintf(char **buf, size_t size, char const *format, va_list va)
{
  int tmp;
  char *tmpbuf;
  int res = -1; // by default this function is not succesful
  if (!size)    // error: invalid size
  {
    errno = EINVAL;
  }
  else
  {
    tmpbuf = (char *)realloc(*buf, size);
    if (tmpbuf) // Memory allocation was successful, print string into buf
    {
      *buf = tmpbuf;
      tmp = vsnprintf(*buf, size, format, va);

      if (tmp >= 0) // vsnprintf was succesful
      {
        res = tmp;
      }
    }
  }
  return res;
}

/*! ubx_asnprintf should work like snprintf, except that it does allocate
the buffer itself.
\param buf    : Will contain the pointer of the allocated buffer
\param size   : The size of the buffer to be allocated
\param format : The format string
\param ...    : Arguments formatted by format string
\return         The return value equals the number of written bytes on
\return         success. On error The return value might is -1.
\return         If the call was successful buf must be freed after usage
*/

int ubx_asnprintf(char **buf, size_t size, char const *format, ...)
{
  int res;
  va_list args;
  va_start(args, format);
  res = ubx_vasnprintf(buf, size, format, args);
  va_end(args);
  return res;
}

/*! ubx_vasprintf should work like vasprintf, except that it does not only run
in GNU source.
\param buf    : Will contain the pointer of the allocated buffer
\param format : The format string
\param va     : The va_list with the arguments formatted by format string
\return         The return value equals the number of written bytes on
\return         success. On error The return value might is -1.
\return         If the call was successful buf must be freed after usage
*/

int ubx_vasprintf(char **buf, char const *format, va_list va)
{
  *buf = NULL; // initialisation required by realloc
  size_t buf_size = 128;
  int tmp;
  bool stayInLoop;
  int res = -1;

  tmp = ubx_vasnprintf(buf, buf_size, format, va);
  if (tmp < 0) // error
  {
    free(*buf);
  }
  if ((size_t)tmp > buf_size) // buffer too small
  {
    // if more bytes could have been written than there is space
    // available in the buffer, the buffer size is increased by the
    // number of bytes that could not have been written and reprint
    // the data to the buffer +1 is for the trailing \0
    buf_size += (size_t)tmp + 1;
    tmp = ubx_vasnprintf(buf, buf_size, format, va);
    if (tmp < 0)
    {
      free(*buf);
    }
    else // the second writeing to buffer worked, exit loop and function
    {
      res = tmp;
    }
  }
  else // everything written to the buffer, exit loop and function
  {
    res = tmp;
  }

  return res;
}

/*! ubx_asprintf should work like asprintf, except that it does not only run
in GNU source.
\param buf    : Will contain the pointer of the allocated buffer
\param format : The format string
\param ...    : Arguments formatted by format string
\return         The return value equals the number of written bytes on
\return         success. On error The return value is -1.
\return         If the call was successful buf must be freed after usage
*/

int ubx_asprintf(char **buf, char const *format, ...)
{
  int res;
  va_list args;
  va_start(args, format);
  res = ubx_vasprintf(buf, format, args);
  va_end(args);
  return res;
}

#endif // if !defined(_GNU_SOURCE) || !defined(__GNUC__)

///////////////////////////////////////////////////////////////////////////////
//! Print a message to logcat with additional debugging information - DONT USE
/*!
Prints beside the message also information about the class and the function
from which the message is originating, as well as, if enabled, file and line
information. Maximal message length: 4096 characters
- DONT USE THIS DIRECTLY. USE \ref UBX_LOG INSTEAD!
\param ll       : Log level of the message in logcat.
Please refer to system/core/include/android/log.h
\param file     : The file from which the message is printed
\param line     : The line from which the message is printed
\param c_func   : The c_func from which the message is printed
\param cpp_func : The cpp_func from which the message is printed
\return   : Pointer to string
*/

int _ubx_log(LCAT_LOGLEVEL pri,
             char const *const file,
             long int const line,
             char const *const c_func,
             char const *const cpp_func,
             char const *const fmt,
             ...)
{
  ((void)file);
  ((void)line);
  int res = -1;
  char *buf = NULL;
  char *filenline = (char *)"";

  va_list args;
  va_start(args, fmt);

  // get the length the string will have. +1 for the terminating \0
  int buf_size = ubx_vasprintf(&buf, fmt, args);
  if (buf_size < 0)
  {
    LOG(
      LOG_ERROR, LOG_TAG, "%s: Error while creating error string '%s'", __func__, strerror(errno));
  }
  else
  {
    char *func_buf = (char *)c_func;
    int cpp_func_len = 0;
    if (cpp_func && *cpp_func)
    {
      // the cpp_func string has the format
      // "[[static] [const] void ]cl::func(a1, a2, ...)"
      // find the last blank before the opening bracket and use it as
      // start of the function string. If it does not exist, (e.g.
      // for constructors) use the beginning of the string as start.
      cpp_func_len = (int)strlen(cpp_func) + 1;

      // find the first blank
      int i = 0;
      int k = 0;

      for (; k < cpp_func_len && cpp_func[k] != '('; ++k)
      {
        if (cpp_func[k] == ' ')
        {
          i = k;
        }
      }

      // if i points to a blank (could also be the first character of a
      // constructor name), take the next character as starting point for
      // the name
      if (i + 1 < cpp_func_len && cpp_func[i] == ' ')
      {
        // Avoid the blank as start character
        ++i;
      }
      else
      {
        // Use the start of the old string as start of the new string
        i = 0;
      }

      cpp_func_len = ubx_asprintf(&func_buf, "%s", cpp_func + i);
      if (func_buf == nullptr)
      {
        LOG(LOG_ERROR, LOG_TAG, "Error in asprintf");
        va_end(args);
        return -1;
      }

      // Did we find the opening brakcet? i is the offset of the new
      // (sub-)string in the original string
      if (k - i < cpp_func_len && func_buf[k - i] == '(')
      {
        func_buf[k - i] = '\0';
      }
    }

    if (cpp_func_len < 0) // error
    {
      func_buf = (char *)c_func;
      LOG(LOG_ERROR,
          LOG_TAG,
          "%s: Error while creating function string "
          "'%s'",
          __func__,
          strerror(errno));
    }

#ifndef NDEBUG
    int filenline_length = ubx_asprintf((char **)&filenline, "%s:%li ", file, line);
    if (filenline_length <= 0)
    {
      filenline = (char *)"";
      LOG(LOG_ERROR,
          LOG_TAG,
          "%s: Error while creating string of file name and line"
          "number function: %s",
          __func__,
          strerror(errno));
    }
#endif // ifndef NDEBUG
    // log priority matching between this function and LOG() has to be done
    // with this switch-case selection, because the LOG_* levels used for
    // LOG() are concatenated to other text with the preprocessor operator
    // "##" and the passed argument is used as string literal...
    // Thus, calling LOG() in a better way by passing the correct priority
    // as a variable to the LOG() function is not possible. The only other
    // option would be to call the functions called by LOG() directly, but
    // but these are not meant to be used in this way.
    switch (pri)
    {
    case LCAT_ERROR:
      res = LOG(LOG_ERROR, LOG_TAG, "%s%s: %s", filenline, func_buf, buf);
      break;
    case LCAT_WARNING:
      res = LOG(LOG_WARN, LOG_TAG, "%s%s: %s", filenline, func_buf, buf);
      break;
    case LCAT_VERBOSE:
      res = LOG(LOG_VERBOSE, LOG_TAG, "%s%s: %s", filenline, func_buf, buf);
      break;
    case LCAT_INFO:
      res = LOG(LOG_INFO, LOG_TAG, "%s%s: %s", filenline, func_buf, buf);
      break;
    case LCAT_DEBUG:
      res = LOG(LOG_DEBUG, LOG_TAG, "%s%s: %s", filenline, func_buf, buf);
      break;
    default:
      // This should never happen
      // lint !e527
      assert(0); // Make sure this fails during debugging
      LOG(LOG_ERROR,
          LOG_TAG,
          "%s: Error while creating printing function "
          "'Invalid Loglevel'",
          __func__);
      res = LOG(LOG_ERROR, LOG_TAG, "INVALID LOGLEVEL! %s%s: %s", filenline, func_buf, buf);
      break;
    }

    free(buf);

    if (cpp_func_len > 0)
    {
      free(func_buf);
    }

#ifndef NDEBUG
    if (filenline_length > 0)
    {
      free(filenline);
    }
#endif // ifndef NDEBUG
  }

  va_end(args);
  return res;
}

///////////////////////////////////////////////////////////////////////////////
//! Lookup a string from a string table
/*!
\param v : Index into string table
\param l : Pointer to table of strings
\param n : Max number of strings in table
\return  : Pointer to string
*/
const char *_strLookup(unsigned int v, const std::vector<std::string> &vec)
{
  if (v < vec.size() && !vec.at(v).empty())
    return vec.at(v).c_str();
  else
    return ublox::log::stringtables::UNKNOWN.c_str();
}

const char *_strLookup(unsigned int v, const std::map<unsigned int, std::string> &map)
{
  const auto it = map.find(v);
  return it != map.end() ? it->second.c_str() : ublox::log::stringtables::UNKNOWN.c_str();
}

///////////////////////////////////////////////////////////////////////////////
//! Lookup a set of strings
/*! Lookup a set of strings from a string table according to flags,
and assemble into a buffer, a pointer to which is returned to the caller.
\param v : Flags  into string table
\param l : Pointer to table of strings
\param n : Max number of strings in table
\return  : pointer to buffer of assembled strings
*/
const char *_strLookupX(unsigned int v, const std::vector<std::string> &vec)
{
  static std::string retVal;
  retVal.clear();
  std::bitset<sizeof(unsigned int) * 8> bits =
    static_cast<std::bitset<sizeof(unsigned int) * 8>>(v);
  for (std::size_t i = 0; i < bits.size() && i < vec.size(); ++i)
  {
    if (bits.test(i))
    {
      auto value = vec.at(i);
      value = (value.empty() || (value == ublox::log::stringtables::UNKNOWN)) ?
                ublox::log::stringtables::UNKNOWN + std::to_string(i) :
                value;
      retVal.append(value + "+");
    }
  }
  retVal.pop_back();
  return retVal.c_str();
}

///////////////////////////////////////////////////////////////////////////////
//! Convert a byte array in a string of hex representations
/*! Convert the byte array of size dataLen pointed at by pData to a string
of hex representations of the contained bytes. The result is stored in a
newly allocated structure stored to pResult (has to be freed after after
a successful call).
\param pResult   : The pointer which will point to a newly allocated
string after a succesful call
\param pData     : The byte array of which the values will be converted
\param dataLen   : The number of bytes stored in the memory pointed to
by pData.
\return          : The number of the characters in the resulting string,
excluding the terminating '\0' on success and -1 on
failure.
*/
std::string byte_array_to_hex_string(unsigned char const *pData, int dataLen)
{
  std::stringstream message;
  if (!pData || dataLen <= 0)
  {
    return message.str();
  }

  for (int i = 0; i < dataLen; i++)
  {
    message << std::setfill('0') << std::setw(2) << std::noshowbase << std::hex
            << static_cast<unsigned int>(pData[i]);
    if (i < (dataLen - 1))
    {
      message << " ";
    }
  }
  return message.str();
}

#ifdef SUPL_ENABLED

const size_t CLog::TIMESTRING_SIZE = 32;
const size_t CLog::BUF_SIZE = 256;

///////////////////////////////////////////////////////////////////////////////
// CMCC compatible logging of files.

///////////////////////////////////////////////////////////////////////////////
//! Constructor
/*!
\param name : Pointer to log path and filename
\param max  : Maximun size (in bytes) of log
*/
CLog::CLog(const char *name, int max, bool verbose)
{
  static const char *env = NULL;
  if (env == NULL)
  {
    env = getenv("EXTERNAL_STORAGE");
    if (env == NULL)
      env = "/mnt/sdcard";
    gid_t gids[] = { AID_SDCARD_RW };
    if (setgroups(sizeof(gids) / sizeof(gids[0]), gids) != 0)
    {
      UBX_LOG(LCAT_ERROR, "cannot get write accress to sdcard %d", errno);
      // gps log
      env = "/data"; // todo remove this line
    }
  }
  /*
  if (setgid(AID_SHELL) != 0)
  {
  UBX_LOG(LCAT_ERROR, "cannot set gid %d %d %d", getgid(), getegid(), errno);
}
if (setuid(AID_SHELL) == -1)
{
UBX_LOG(LCAT_ERROR, "cannot set uid %d %d %d", getuid(), geteuid(), errno);
}
int s = getgroups(0,NULL);
gid_t* g = new gid_t[s];
getgroups(s, g);
UBX_LOG(LCAT_DEBUG, "gids:");
while (s --)
UBX_LOG(LCAT_DEBUG, "gid %d", *g++);
*/
  snprintf(m_name, NAME_SIZE, "%s/%s", env, name);
  m_max = max;
  m_verbose = verbose;
}

///////////////////////////////////////////////////////////////////////////////
//! Format and write to log file
/*!
\param code : Logging code
\param fmt  : Pointer to string containing formatting (as per a printf)
\param ...  : Variable parameters to log (as per a printf)
*/
void CLog::write(unsigned int code, const char *fmt, ...)
{
  if (!CUbxGpsState::getInstance()->getCmccLogActive())
  {
    return;
  }
  char buf[BUF_SIZE];
  char tmp[TIMESTRING_SIZE];
  int str_len = snprintf(buf, BUF_SIZE, "[%s] 0x%08X:", timestamp(tmp), code);
  if (str_len < 0)
  {
    UBX_LOG(LCAT_ERROR,
            "Could not write timesstamp and error code to buf:"
            " '%s'",
            strerror(errno));
    return;
  }
  else if ((size_t)str_len >= BUF_SIZE) //>= because i does not include '\0'
  {
    UBX_LOG(LCAT_ERROR,
            "Could not write timesstamp and error code to buf:"
            " 'Too many characters (%i) in string'",
            str_len);
    return;
  }
  int tot_len = str_len;
  va_list args;
  va_start(args, fmt);
  // lint -e{530} remove Symbol 'args' (line 266) not initialized
  str_len = vsnprintf(&buf[str_len], BUF_SIZE - (size_t)str_len, fmt, args);
  va_end(args);

  if (str_len < 0)
  {
    UBX_LOG(LCAT_ERROR,
            "Could not write arguments to buf with formatting"
            " '%s': '%s'",
            fmt,
            strerror(errno));
    return;
  }
  else if ((size_t)str_len >= (BUF_SIZE - tot_len)) //>= because i does not include '\0'
  {
    UBX_LOG(LCAT_ERROR,
            "Could not write arguments to buf with formatting"
            " '%s': 'Too many characters (%i) in string'",
            fmt,
            str_len);
    return;
  }

  tot_len += str_len;

  if (m_verbose)
    UBX_LOG(LCAT_DEBUG, "file='%s' data='%s' size=%d", m_name, buf, tot_len);
  if ((size_t)tot_len < BUF_SIZE - 3) // There is space for the three end characters
  {
    buf[tot_len++] = '\r';
    buf[tot_len++] = '\n';
    buf[tot_len] = '\0';
  }
  else // No space for the three end characters - truncate string
  {
    buf[BUF_SIZE - 3] = '\r';
    buf[BUF_SIZE - 2] = '\n';
    buf[BUF_SIZE - 1] = '\0';
    UBX_LOG(LCAT_ERROR,
            "There was not enough space for the tree end "
            "characters. Truncating the string!");
  }

  writeFile(buf, tot_len);
}

///////////////////////////////////////////////////////////////////////////////
//! Write to log file
/*!
\param pBuf : Pointer to buffer to write
\param len  : Number of bytes to write
*/
void CLog::writeFile(const char *pBuf, int len)
{
  FILE *pFile = fopen(m_name, "a");
  if (pFile == NULL)
  {
    // failed
    UBX_LOG(LCAT_ERROR, "Can not open '%s' file : %i", m_name, errno);
    return;
  }

  int size = ftell(pFile);
  if (size > m_max)
  {
    // Max file size reached, so recreate file with zero size
    pFile = freopen(m_name, "w", pFile);
    UBX_LOG(LCAT_WARNING, "file='%s' resize", m_name);
  }

  if (pFile == NULL)
  {
    // failed
    UBX_LOG(LCAT_ERROR, "Can not reopen '%s' file : %i", m_name, errno);
    return;
  }

  fwrite(pBuf, 1, (unsigned int)len, pFile);
  fclose(pFile);
}

///////////////////////////////////////////////////////////////////////////////
//! Generate a time stamp for logging
/*!
\param tmp : Pointer to string place time stamp text into. Must point to an
array with a minimm of TIMESTRING_SIZE elements.
If NULL is passedas argument, a pointer to a static char array
is returned, which makes the function not thread-safe and
overwrites the content of the array if the function is called
with NULL as argument again.
\return    : Pointer to string containing time stamp. (Either tmp or a
static array.
*/
const char *CLog::timestamp(char *tmp)
{
  static char buf[TIMESTRING_SIZE];
  struct timeval tv;
  gettimeofday(&tv, NULL);
  char *pBuf = tmp ? tmp : buf;

  struct tm st;
  int res;
  gmtime_r(&tv.tv_sec, &st);
  res = snprintf(pBuf,
                 TIMESTRING_SIZE,
                 "%04d%02d%02d%02d%02d%02d.%02d",
                 st.tm_year + 1900,
                 st.tm_mon + 1,
                 st.tm_mday,
                 st.tm_hour,
                 st.tm_min,
                 st.tm_sec,
                 (int)(tv.tv_usec / 10000));
  if (res < 0)
  {
    UBX_LOG(LCAT_ERROR, "Can not generate a timestring: %s", strerror(errno));
    pBuf[0] = '\0';
  }
  return pBuf;
}

///////////////////////////////////////////////////////////////////////////////
//! Destructor
/*!
*/
CLog::~CLog() {}

///////////////////////////////////////////////////////////////////////////////
//! Log text.
/*! Log text, but add a time stamp to the body of the text
\param code : Logging code
\param txt  : Pointer to text to log
*/
void CLog::txt(int code, const char *pTxt)
{
  char tmp[TIMESTRING_SIZE];
  write((unsigned int)code, "%s %s", timestamp(tmp), pTxt);
}

///////////////////////////////////////////////////////////////////////////////
// Static data
static CLog s_logMessages("SUPL-MESSAGE.LOG", 128 * 1024, false);

///////////////////////////////////////////////////////////////////////////////
//! Transfer a large memory buffer to  the log
/*! Message buffer may be too big to log in one hit, so parse the message
using 0x0A characters to split into multiple lines. Then log each line.
\param pMsg : Pointer to message buffer to log
\param size : Size of buffer
*/
static void renderMemStreamToLog(char *pBuf, size_t size)
{
  unsigned int i = 0;
  unsigned int start = i;
  while (i < size)
  {
    if (pBuf[i] == 0x0A)
    {
      pBuf[i] = 0;
      if (CUbxGpsState::getInstance()->getSuplMsgToFile())
      {
        s_logMessages.writeFile(&pBuf[start], (int)(i - start));
        s_logMessages.writeFile("\r\n", 2);
      }

      if (CUbxGpsState::getInstance()->getLogSuplMessages())
      {
        UBX_LOG(LCAT_VERBOSE, "%s", &pBuf[start]);
      }

      start = i + 1;
    }
    i++;
  }
}

///////////////////////////////////////////////////////////////////////////////
//! Log a separator
/*!
*/
static void logSep(void)
{
  char buffer[200];
  int len = sprintf(buffer,
                    "=================================================="
                    "==============================\n");
  renderMemStreamToLog(buffer, (unsigned int)len);
}

///////////////////////////////////////////////////////////////////////////////
//! Decode a Supl message to text and send to log
/*!
\param pMsg     : Pointer to Supl message to decode to log
\param incoming : Direction of message
*/
void logSupl(const struct ULP_PDU *pMsg, bool incoming)
{
  if (CUbxGpsState::getInstance()->getLogSuplMessages() ||
      CUbxGpsState::getInstance()->getSuplMsgToFile())
  {
    logSep();

    char buffer[200];
    int len = sprintf(buffer, "%s\n", incoming ? "<------- From Server" : "-------> To Server");
    renderMemStreamToLog(buffer, (unsigned int)len);

    char *pBuf = NULL;
    size_t size = 0;
    FILE *pFile = open_memstream(&pBuf, &size); // Supl print message needs a file stream
    if (pFile == nullptr)
    {
      // failed
      UBX_LOG(LCAT_ERROR, "Error: Can not open memstream");
      return;
    }
    asn_fprint(pFile, &asn_DEF_ULP_PDU, pMsg);
    fclose(pFile);
    renderMemStreamToLog(pBuf, size); // Now transfer from memory to Android log
    free(pBuf);
    logSep();
  }
}

///////////////////////////////////////////////////////////////////////////////
//! Decode a RRLP message to text and send to log
/*!
\param pMsg : Pointer to RRLP message to decode to log
*/
void logRRLP(const PDU_t *pRrlpMsg, bool incoming)
{
  if (CUbxGpsState::getInstance()->getLogSuplMessages() ||
      CUbxGpsState::getInstance()->getSuplMsgToFile())
  {
    logSep();

    char buffer[200];
    int len = sprintf(buffer, "RRLP Payload\n");
    renderMemStreamToLog(buffer, (unsigned int)len);

    len = sprintf(buffer, "%s\n", incoming ? "<------- From Server" : "-------> To Server");
    renderMemStreamToLog(buffer, (unsigned int)len);

    char *pBuf = NULL;
    size_t size = 0;
    FILE *pFile = open_memstream(&pBuf, &size); // RRLP print message needs a file stream
    if (pFile == nullptr)
    {
      // failed
      UBX_LOG(LCAT_ERROR, "Error: Can not open memstream");
      return;
    }
    asn_fprint(pFile, &asn_DEF_PDU, pRrlpMsg);
    fclose(pFile);
    renderMemStreamToLog(pBuf, size); // Now transfer from memory to Android log
    free(pBuf);
    logSep();
  }
}

///////////////////////////////////////////////////////////////////////////////
// Instances for each log file
CLog logGps("GPS.LOG", 64 * 1024);
CLog logAgps("A-GPS.LOG", 32 * 1024);

#endif
