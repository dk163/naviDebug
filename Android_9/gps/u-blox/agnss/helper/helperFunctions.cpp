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

/*! \file
    Definitions of functions used in the AGNSS part of the driver
*/

#include "helperFunctions.h"
#include <algorithm>
#include <assert.h>
#include <climits>
#include <errno.h>
#include <iterator>
#include <limits.h>
#include <list>
#include <map>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

namespace
{
  //! Translates LEG Data IDs to Text
  std::map<unsigned int, std::string> s_legDataIdToTxt = {
    { 0x31, "EPH" },  //
    { 0x30, "ALM" },  //
    { 0x02, "HUI" },  //
    { 0x01, "INI" }, //
  };

  //! Translates MGA Data IDs to text
  std::vector<std::string> s_mgaDataIdToTxt = {
    "EPH",  //
    "ALM",  //
    "TIMEOFFSET",  //
    "HEALTH",  //
    "UTC",  //
    "IONO" //
  };

  //! Translates MGA-INI numbers to text
  std::vector<std::string> s_mgaIniIdToTxt = {
    "MGA-INI-POS_XYZ",  //
    "MGA-INI-POS_LLH",  //
    "MGA-INI-TIME_UTC",  //
    "MGA-INI-TIME_GNSS", //
    "MGA-INI-CLKD",     //
    "MGA-INI-FREQ", //
    "MGA-INI-EOP", //
  };

  //! Translates GNSS IDs to text
  std::vector<std::string> s_gnssIdToTxt = { "GPS", "SBAS", "GAL", "BDS", "IMES", "QZSS", "GLO" };

  //! Translates AGNSS Service IDs to text
  std::vector<std::string> s_agnssServiceIdToTxt = {
    "Time", "Position", "Receiver Aiding State", "Offline", "Online"
  };

  //! Translates AGNSS Action IDs to text
  std::vector<std::string> s_agnssActionIdToTxt = { "Transfer", "Download", "Poll" };
#ifdef NDEBUG
  auto assertTrue = [](bool const /*v*/) {};
#else
  auto assertTrue = [](bool const v) { assert(v == true); };
#endif
}

//! Lookup a string from a string table
/*!
    \param v : Index into string table
    \param l : Pointer to table of strings
    \param n : Max number of strings in table
    \return  : Pointer to string
*/
extern const char *_strLookup(unsigned int v, const std::vector<std::string> &vec);
extern const char *_strLookup(unsigned int v, const std::map<unsigned int, std::string> &map);
#ifndef _LOOKUPSTR
#define _LOOKUPSTR(v, t) _strLookup(v, t)
#endif

char const *gnssIdToString(unsigned int gnssId) { return _LOOKUPSTR(gnssId, s_gnssIdToTxt); }

char const *aidingDataTypeToString(unsigned int dataType)
{
  return _LOOKUPSTR(dataType, s_mgaDataIdToTxt);
}

char const *mgaIniDataTypeToString(unsigned int iniType)
{
  return _LOOKUPSTR(iniType, s_mgaIniIdToTxt);
}

char const *agnssServiceTypeToString(unsigned int iniType)
{
  return _LOOKUPSTR(iniType, s_agnssServiceIdToTxt);
}

char const *agnssActionToString(unsigned int iniType)
{
  return _LOOKUPSTR(iniType, s_agnssActionIdToTxt);
}

char const *ubxIdsToString(unsigned int classId, unsigned int msgId)
{
  if (classId != 0x0B) // Aiding
  {
    msgId = UINT_MAX;
  }

  return _LOOKUPSTR(msgId, s_legDataIdToTxt);
}

/*! Log the provided message to the provided log function if eixsting.
    Note: The combined message (format string expanded with the additionally
    provided arguments) can not exceed MAX_LOG_LENGTH including the string
    termination character.

    \param func        : Function to which the message should be logged
    \param prio        : Priority of the logged message
    \param format      : The format string is a character string, to which the
                         same rules have to apply as to the one provided e.g.
                         to \ref printf()
    \param ...         : Arguments formatted by format string
    \return              The return value equals the number of logged bytes,
                         or if exceeding MAX_LOG_LENGTH the number of bytes
                         that should have been logged. If 0 nothing was logged
                         due to func being NULL. If negative an error occured.
                         In this case errno should indicate the type of problem
*/
int logTo(LOG_FUNC_p func, LOG_PRIO_t prio, char const *format, ...)
{
  int result = 0; // If nothing can be printed
  if (func)
  {
    char msg[MAX_LOG_LENGTH];

    // Construct the final message that should be logged in msg
    va_list args;
    va_start(args, format);
    result = vsnprintf(msg, sizeof(msg) / sizeof(msg[0]), format, args);
    va_end(args);

    // If msg could be constructed, print it
    if (result > 0)
    {
      (*func)(prio, msg);
    }
  }
  return result;
}

/*! This function will verify if the passed data contains a valid
    UBX message starting at the first byte of the buffer. This
    function checks the form of the message, not its actual
    content.

    \param pBuffer    : The buffer which should be checked for a
                        UBX message at its beginning. Must not be NULL
    \param iSize      : The size of the data pointed to by pBuffer in bytes.
                        Must not be 0.
    \param ignTrailCh : Should trailing characters be ignored?
    \return             On success the length of the found message in bytes
                        Otherwise an error code <0
*/
ssize_t isUbxMessage(const unsigned char *pBuffer, size_t iSize, bool ignTrailCh /* = false */)
{
  if (iSize < 8) // minimum size of valid UBX
    return HELPER_ERR_ARG_TOO_SHORT;

  if (pBuffer[0] != 0xb5 || pBuffer[1] != 0x62)
    return HELPER_ERR_UNKNOWN_FORMAT;

  U2 iLength = (U2)(((U2)pBuffer[4]) + (((U2)pBuffer[5]) << 8));

  if (iLength + (size_t)8 > iSize)
    return HELPER_ERR_ENTRY_TOO_SHORT;

  // If the exact length must be matched, make sure it does!
  if (!ignTrailCh && iSize != iLength + (size_t)8)
    return HELPER_ERR_ENTRY_TOO_SHORT;

  // calculate the cksum
  unsigned char ckA = 0;
  unsigned char ckB = 0;

  for (U2 i = 2; i < iLength + 6; ++i)
  {
    ckA += pBuffer[i];
    ckB += ckA;
  }

  // check the cksum
  if (pBuffer[iLength + 6] != ckA || pBuffer[iLength + 7] != ckB)
    return HELPER_ERR_CHECKSUM_UBX;

  return iLength + 8; //+ frame size
}

/*! This function will iterate through the buffer passed to it and
    verify that all containing messages are valid UBX messages and
    are in the allowed list. Additionally it will make sure that all
    bytes contained belong to a UBX message.

    \param buf        : The buffer containing the UBX messages
                        that has to be checked. Must not be NULL
                        and may only contain bytes belonging to
                        one or more UBX messages
    \param size       : The size of the data pointed to by buf in bytes.
                        Must not be 0.
    \param allowed    : List of message types that are allowed to be
                        processed.
    \param allowed_num: Number of message types in allowed
    \param msgs       : Optional argument. The pointer which is referred to
                        by the address passed will be made to point
                        to a newly allocated array of BUFC_t structures
                        which will contain for each message found in buf
                         a pointer to the beginning of the message  and its
                        size in bytes on success. This must be freed on
                        success.
    \return             On success the number of valid messages found. On
                        error a negative number. 0 is not a valid return value
*/
ssize_t verifyUbxMsgsBlock(unsigned char const *buf,
                           size_t size,
                           UBX_MSG_TYPE const *allowed,
                           size_t allowed_num,
                           BUFC_t **msgs)
{
  if (!buf || size <= 0 || size > (size_t)SSIZE_MAX)
    return HELPER_ERR_INVALID_ARGUMENT;

  // create a copy of the data to parse
  auto tmpBuf = std::vector<unsigned char>();
  tmpBuf.resize(size);
  ssize_t tmpSize = (ssize_t)size;

  std::copy(buf, buf + size, tmpBuf.begin());

  BUFC_t *tmpMsgs = NULL;
  ssize_t result = 0;
  bool endLoop = false;

  do
  {
    ssize_t msg_length = isUbxMessage(tmpBuf.data(), tmpSize, true);
    if (msg_length < 0)
    {
      // The beginning of the buffer does not contain a ubx
      // message. abort
      result = msg_length;
    }
    else // Valid ubx message found
    {
      // Is there a list of allowed ubx messages? If yes make
      // sure it is obeyed. If not break here
      if (allowed && allowed_num &&
          !isAllowedUbxMsg(tmpBuf.data(), msg_length, allowed, allowed_num))
      {
        result = HELPER_ERR_ENTRY_NOT_ALLOWED;
      }
      else // Msg is allowed or no list of allowed messages provided
      {
        // If msgs are defined, this message has to be filled in there
        if (msgs)
        {
          //lint !e449
          BUFC_t *newTmpMsgs = (BUFC_t *)realloc(tmpMsgs, (result + 1) * sizeof(BUFC_t));
          if (newTmpMsgs)
          {
            tmpMsgs = newTmpMsgs;
            tmpMsgs[result].i = msg_length;
            tmpMsgs[result].p = &buf[size - tmpSize];
          }
          else
          {
            // Clean up allocated memory later on
            result = HELPER_ERR_OUT_OF_MEMORY;
          }
        }

        // Did an error occure until now?
        if (result >= 0)
        {
          // There is an additional valid ubx message
          // Make sure the counter is correct as well as the new length
          ++result;
          tmpSize -= msg_length;

          // This should be guaranteed by isUbxMessage()
          assert(tmpSize >= 0);

          // Remove the ubx message detected last
          tmpBuf.erase(tmpBuf.begin(), tmpBuf.begin() + msg_length);

          // No bytes left and all bytes are valid ubx messages -> Success!
          if (!tmpSize)
            endLoop = true;
        }
      }
    }

    // If result is set to an error state
    if (result < 0)
      endLoop = true;

  } while (!endLoop);

  // If msgs are enabled and the function was successful,
  // provide the data to the caller
  if (result > 0 && msgs)
  {
    //lint -esym(449, tmpMsgs)
    *msgs = tmpMsgs;
  }
  else // If the function failed or msgs are not enabled free the tmpMsgs pointer
  {
    //lint -e(438)
    free(tmpMsgs);
  }

  return result;
}

/*! This function will check if the passed data contains UBX messages
    whitelisted. This function does NOT verify if the passed message is
    in valid UBX form or if the checksum is correct.
    It only checks for valid class id and message id in regard to aiding.

    \param buf        : A pointer to message that should be checked for
                        being whitelisted. Must not be NULL
    \param size       : The size of the data in bytes pointed to by buf
                        Must not be 0.
    \param allowed    : Class- and Message- Ids of UBX messages that
                        are whitelisted. May be NULL, in which case
                        all messages are allowed
    \param allowed_num: The number of entries in the allowed array
    \param enWildCards: Enable 0xFF as message and class ID in the
                        allowed to act as wild card. This can
                        be used either to allow all messsages or to
                        allow all message of a certain class.
    \return             On success 'true'. On failure 'false'
*/
bool isAllowedUbxMsg(unsigned char const *buf,
                     int size,
                     UBX_MSG_TYPE const *allowed,
                     size_t allowed_num,
                     bool enWildCards /* = false */)
{
  if (!buf || size < 8 || !allowed || !allowed_num)
    return false;

  bool allowedMsg = false;

  for (size_t i = 0; i < allowed_num && !allowedMsg; ++i)
  {
    // Check if the message is in the list of allowed messages
    if (buf[2] != 0xFF &&
        buf[3] != 0xFF // No messages with class or message Id 0xFF exist, we use them as wildcards
        &&
        buf[2] == allowed[i].clsId && buf[3] == allowed[i].msgId)
    {
      allowedMsg = true;
    }
    else if (enWildCards) // Are wild cards allowed?
    {
      if ((allowed[i].clsId == 0xFF &&
           allowed[i].msgId == 0xFF) // All messages of all classes allowed
          ||
          (buf[2] == allowed[i].clsId &&
           allowed[i].msgId == 0xFF)) // All messages of a class allowed
      {
        allowedMsg = true;
      }
    }
  }

  return allowedMsg;
}

/*! This function will verify if the passed data contains a valid
    ALP structure starting at the first byte. If this is the case
    the header of it will be copied to the provided structure.

    \param msg        : A pointer to the buffer from which the data should
                        copied to the header structure if it is valid ALP
                        data. Must not be NULL.
    \param size       : The size of the data in bytes pointed to by pData
                        Must not be 0.
    \param header     : A pointer to the structure to which the data
                        should be copied on success.
    \return             On success 0. On failure an error code <0
*/
int verifyAlpData(const unsigned char *msg, unsigned int size, ALP_FILE_HEADER_t *header)
{
  if (!msg || !size)
    return HELPER_ERR_INVALID_ARGUMENT;

  if (size < (int)sizeof(ALP_FILE_HEADER_t))
    return HELPER_ERR_ARG_TOO_SHORT;

  ALP_FILE_HEADER_t tmp;
  tmp = *(reinterpret_cast<const ALP_FILE_HEADER_t *>(msg));

  if (tmp.magic != 0x015062b5)
    return HELPER_ERR_INVALID_MAGIC;
  else if (tmp.size * 4 != size)
    return HELPER_ERR_ENTRY_TOO_SHORT;

  if (header)
    *header = tmp;

  return 0; //success
}

/*! This function will read from the provided file descriptor until the
    no more bytes are available in the buffer. This can only be used
    for devices which support the FIONREAD argument for ioctl.

    \param fd         : The file descriptor from which data should be read
    \param buf        : The pointer which will be set to data buffer. This
                        will only be changed, if the return value is
                        bigger than 0
    \param maxSize    : The maximum number of bytes that will be read

    \return             On success the number of bytes written to buf.
                        On failure an error code <0. 0 If no bytes
                        are available
*/
ssize_t availRead(int fd, unsigned char **buf, size_t maxSize /* = 0 */)
{
  if (fd < 0 || !buf || maxSize > (size_t)SSIZE_MAX)
    return -1;

  ssize_t result = -1;

  int bytes_available = 0;

  if (ioctl(fd, FIONREAD, &bytes_available) == 0)
  {
    if (bytes_available > 0)
    {
      size_t size = maxSize;

      // If the max-size has not been set or less
      // bytes are available, take whatever is in
      // the buffer
      if (size == 0 || size > (unsigned)bytes_available)
        size = bytes_available;

      unsigned char *pTmp = (unsigned char *)malloc(size);
      // malloc succeeded?
      if (pTmp)
      {
        // get the data
        result = read(fd, pTmp, size);

        if (result > 0)
        {
          *buf = pTmp;
        }
        else
        {
          int errVal = errno;
          free(pTmp);
          errno = errVal;
        }
      }
    }
    else
    {
      result = 0;
    }
  }

  return result;
}

/*! This function will read from the provided file descriptor until the
    provided byte sequence has been found (in which case it finishes
    successfully), the specified timeout has elapsed (always a failure)
    or the provided buffer is full (which is a failure if a sequence
    has been provided and a success otherwise)

    \param fd         : The file descriptor from which data should be read
    \param buf        : The buffer to which received data will be written
    \param bufSize    : The size of the provided buffer
    \param stopSeq    : The sequence which will indicate the end of
                        a successful transfer. Can be NULL if no sequence
                        has to be found
    \param stopSeqSize: The size of the provided sequence or 0 if none
                        provided
    \param timeoutMs  : The time in milliseconds after which the operation
                        is considered as having failed if the required
                        sequence has not been received yet. If instead
                        of a positive value, a negative value is provided
                        the timeout is infinite. If the value provided is
                        zero, it will be tried to retrieve all data in
                        one go and return immediately.

    \return             On success the number of bytes written to buf.
                        On failure an error code <0
*/

ssize_t extRead(int fd,
                unsigned char *buf,
                size_t bufSize,
                unsigned char const *stopSeq,
                size_t stopSeqSize,
                int timeoutMs)
{
  if (fd < 0 || !buf || bufSize < stopSeqSize || (stopSeq && stopSeqSize == 0))
    return -1;

  size_t iSize = 0;
  bool con_ended = false;
  bool success = false;
  bool useTimeout = false;
  struct pollfd pfd;
  pfd.fd = fd;
  pfd.events = POLLIN;

  // Is this a proper timeout or a value
  // or is it just a signalisation to
  // either poll or wait indefinitely?
  if (timeoutMs > 0)
    useTimeout = true;

  // Get the current monotonic system time
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const int64_t startTimeMs = (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;

  // As long as the buffer is not full,
  // the end character has not been received,
  // and the timeout has not elapsed try to get
  // the next byte
  while (!con_ended)
  {
    // if no bytes arrive within timeoutMs
    // we consider the connection dropped.
    // Is there data in arriving to the
    // input buffer within this time limt?
    if (poll(&pfd, 1, timeoutMs) > 0)
    {
      // If an actual timeout is used (not negative values
      // or zero), the timeout has to be adjusted and reduced
      // by the amount of ms spent in poll()
      if (useTimeout)
      {
        int64_t currentTimeMs;
        // Get the current monotonic system time and
        // check how much time has been spent in poll
        // (must be lower than timeout_ms to end up here)
        // and use the new timeout for the next call
        clock_gettime(CLOCK_MONOTONIC, &ts);
        currentTimeMs = (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
        timeoutMs -= (int)(currentTimeMs - startTimeMs);

        if (timeoutMs <= 0) // If this is 0 by now, the time is up
          con_ended = true;
      }

      // Did we actually get the right POLLIN event?
      // If yes check how many bytes are available
      int bytes_available;
      if (pfd.revents & POLLIN && ioctl(fd, FIONREAD, &bytes_available) == 0 && bytes_available > 0)
      {
        // Get all bytes in the buffer
        // - one at a atime - until the end
        // end-sequence is found (if specified),
        // the buffer is full or all the bytes
        // available have been read
        int iAvail = 0;
        do
        {
          // Yes. Get one byte
          ssize_t ret = read(fd, buf + iSize, 1);
          if (ret < 0)
          {
            // Is this a nonblocking interface and
            //we simply have to try again?
            if (errno != EAGAIN && errno != EWOULDBLOCK)
              con_ended = true;
          }
          else if (ret == 1)
          {
            ++iSize;
            ++iAvail;

            // If there is a stop sequence
            // check if it has arrived
            if (stopSeq && iSize >= stopSeqSize)
            {
              // Did we get our stop sequence?
              if (memcmp(&(buf[iSize - stopSeqSize]), stopSeq, stopSeqSize) == 0)
              {
                // End of header reached
                con_ended = true;
                success = true;
              }
            }
          }

          // If the buffer is full, end this
          if (!con_ended && (ssize_t)bufSize - (ssize_t)iSize <= 0)
          {
            // This is a failure if a
            // sequence was provided,
            // but a success if not
            if (!stopSeq)
              success = true;

            con_ended = true;
          }
        } while (!con_ended && iAvail < bytes_available);
      }
    }
    else
    {
      // Error or Timeout: No response. terminate
      con_ended = true;
    }
  }

  // Did this end successfully?
  ssize_t result = -1;

  if (success)
    result = iSize;

  return result;
}

/*! Calculate the 8-bit Fletcher checksum for the specified buffer
    according to RFC 1145. The first part of the checksum (A) is
    stored in the 8 most significant bits of the returned value,
    while the second part of the checksum (B) is stored in the
    8 leas significant bits of the returned value.

    \param data       : A pointer to the data for which the checksum
                        should be calculated. Must not be NULL.
    \param size       : The number of bytes contained in data. Must
                        not be 0.
    \param chk        : Checksum of previous partial calculation.
                        Optional
    \return             The calculated checksum.
*/
uint16_t calculateChecksum(unsigned char const *const data, size_t size, uint16_t const *chk)
{
  if (!data || !size)
    return 0;

  unsigned char ckA = 0;
  unsigned char ckB = 0;
  if (chk)
  {
    ckA = *chk >> 8;
    ckB = *chk;
  }

  for (size_t i = 0; i < size; i++)
  {
    ckA += data[i];
    ckB += ckA;
  }

  uint16_t result = (ckA << 8) | ckB;

  return result;
}

/*! Encode an array of buffers into a raw buffer with the following format:
    -----------------------------------------------------------------------
    | Name:                   "ubxagnssf" // no \0
    | Encoder Version:        \<uint8_t\>
    | Nr. Entries [N]:        \<int8_t\>    // Must be positive number
    | Bytes in Entry 0:       \<uint32_t\>  // max 1023 KiB (0xFFFFF)
    |        .
    |        .
    |        .
    | Bytes in Entry N-1:     \<uint32_t\>
    | Entry 0:                \<unsigned char[Bytes in Entry[0]]\>
    |        .
    |        .
    |        .
    | Entry N-1:              \<unsigned char[Bytes in Entry[N-1]]\>
    | Checksum:               \<uint16_t\>  // Including Name up
    |                                     // to and including Entry N-1
    -----------------------------------------------------------------------

    \param raw             : The pointer which will refer to the
                             encoded data will be assigned to to pointer
                             of which the address is passed to with this
                             parameter. This buffer has to be freed
                             by the caller, if the function was successful.
    \param bufs            : The buffers of which each will be written
                             as a separate entry to the file. None of the
                             containing data should exceed 1023 KiB. Empty
                             buffers (pointer==NULL and size==0) are allowed
                             and will be stored so that the emtpy buffer.
    \param numBufs         : Number of BUF_t structures in bufs.
                             At least one (if empty) buffer has to be encoded
                             can be retrieved while decoding again.

    \return                  The number of bytes written to raw on success
                             and one of these  negative numbers on failure:
                             \ref HELPER_ERR_UNKNOWN_ERROR
                             \ref HELPER_ERR_INVALID_ARGUMENT
                             \ref HELPER_ERR_ENTRY_TOO_LONG
                             \ref HELPER_ERR_INVALID_ENTRY
                             \ref HELPER_ERR_OUT_OF_MEMORY
                             \ref HELPER_ERR_BUFFER_SIZE_EXCEEDED

*/
ssize_t encodeAgnssf00(unsigned char **raw,
                       BUF_t const *bufs,
                       int8_t numBufs,
                       LOG_FUNC_p logFunc /* = NULL */)
{
  //----------------------------------------------------------------------
  // Guards
  if (!raw || !bufs || numBufs <= 0)
  {
    logTo(logFunc,
          ERROR,
          "One of these arguments is invalid (0 or NULL): %p %p %i",
          raw,
          bufs,
          numBufs);
    return HELPER_ERR_INVALID_ARGUMENT;
  }

  for (int8_t i = 0; i < numBufs; ++i)
  {
    if (bufs[i].i > 0xFFFFF) // Not more than 1023 KiB per block
    {
      logTo(
        logFunc, ERROR, "Encoding of buffer %i not possible as it is too long (%i)", i, bufs[i].i);
      return HELPER_ERR_ENTRY_TOO_LONG;
    }
  }
  // End Guards
  //----------------------------------------------------------------------

  ssize_t result = HELPER_ERR_UNKNOWN_ERROR; // Default error
  uint32_t bufSizes[0x7F]{};
  char const UBXAGNSSF[] = "ubxagnssf";
  uint16_t checksum = 0;
  uint8_t version = 0x00;                        // Version of the decoder
  size_t rawSize = sizeof(checksum)              // Reserve space for checksum
                   + strlen(UBXAGNSSF)           // Reserve space for file id string
                   + sizeof(version)             // Reserve space for version number
                   + sizeof(numBufs)             // Reserve space for no of entries
                   + numBufs * sizeof(uint32_t); // Reserve space for table of entries

  // Iterate through buffers and check buffer size and make
  // sure that if the size is zero the pointer is NULL
  // Also: Calculate the necessary raw buffer size
  for (int8_t i = 0; i < numBufs; ++i)
  {
    if ((!bufs[i].p && bufs[i].i) || (bufs[i].p && !bufs[i].i))
    {
      logTo(logFunc,
            ERROR,
            "Buffer entry %i either has a defined size and a"
            " null-pointer or a valid pointer but zero-size: %p %i",
            i,
            bufs[i].p,
            bufs[i].i);
      return HELPER_ERR_INVALID_ENTRY;
    }

    bufSizes[i] = bufs[i].i;
    rawSize += bufSizes[i];
  }

  // Make sure rawSize is within the limit
  if (rawSize <= (size_t)SSIZE_MAX)
  {
    // Allocate Buffer for encoded (raw) data
    unsigned char *tmpRaw = (unsigned char *)malloc(rawSize);
    if (tmpRaw)
    {
      // Copy file signature type to raw buffer
      size_t offset = 0;
      memcpy(tmpRaw, UBXAGNSSF, strlen(UBXAGNSSF));
      offset += strlen(UBXAGNSSF);

      // Copy version of the encoder to the raw buffer
      memcpy(tmpRaw + offset, &version, sizeof(version));
      offset += sizeof(version);

      // Copy number of entries to the raw buffer
      memcpy(tmpRaw + offset, &numBufs, sizeof(numBufs));
      offset += sizeof(numBufs);

      // Copy entry sizes to table in raw buffer
      memcpy(tmpRaw + offset, bufSizes, numBufs * sizeof(uint32_t));
      offset += numBufs * sizeof(uint32_t);

      // Add data blocks and ignore empty values
      for (int8_t i = 0; i < numBufs; ++i)
      {
        if (bufs[i].p)
        {
          assert(bufs[i].i);
          memcpy(tmpRaw + offset, bufs[i].p, bufs[i].i);
          logTo(logFunc,
                DEBUG,
                "Encoded buffer %i (%p) at offset %i (%p) with size %i",
                i,
                bufs[i].p,
                offset,
                tmpRaw + offset,
                bufs[i].i);
          offset += bufs[i].i;
        }
      }

      // Calculate checksum and copy to raw buffer
      checksum = calculateChecksum(tmpRaw, rawSize - sizeof(checksum));
      memcpy(tmpRaw + offset, &checksum, sizeof(checksum));
      offset += sizeof(checksum);
      if (offset != rawSize)
      {
        return HELPER_ERR_UNKNOWN_ERROR;
      }

      // Copy tmpRaw pointer to final location and make sure
      // the right size is returned
      *raw = tmpRaw;
      result = rawSize;
      logTo(
        logFunc, INFO, "Encoded %i buffers in %i size of memory at %p", numBufs, rawSize, tmpRaw);
    }
    else
    {
      logTo(logFunc,
            ERROR,
            "While trying to allocate the raw buffer of size %i,"
            " the operation run out of memory",
            rawSize);
      result = HELPER_ERR_OUT_OF_MEMORY;
    }
  }
  else
  {
    logTo(logFunc,
          ERROR,
          "The raw buffer size of %i exceeds the maxium of %i",
          rawSize,
          (size_t)SSIZE_MAX);
    result = HELPER_ERR_BUFFER_SIZE_EXCEEDED;
  }

  return result;
}

/*! Decode an array of buffers from a raw buffer with the format specified
    in \ref encodeAgnssf00
    \param raw             : The pointer to a memory location which contains
                             the data to decode
    \param rawSize         : The number of bytes in raw
    \param bufs            : The buffers to which the decoded data should
                             be written. The pointers put into these
                             structures have to be freed by the caller.
    \param numBufs         : Number of BUF_t structures in bufs. At least
                             one has to be passed. If not enough bufs are
                             passed to store all data, the function will
                             fail.

    \return                  The number BUF_t structures filled on success
                             and one of these negative numbers on failure:
                             \ref HELPER_ERR_INVALID_ARGUMENT
                             \ref HELPER_ERR_UNKNOWN_FORMAT
                             \ref HELPER_ERR_CHECKSUM_UBX
                             \ref HELPER_ERR_UNSUPPORTED_VER
                             \ref HELPER_ERR_INVALID_ENTRY_NUM
                             \ref HELPER_ERR_ENTRY_TOO_LONG
                             \ref HELPER_ERR_ARG_TOO_SHORT
                             \ref HELPER_ERR_OUT_OF_MEMORY
*/
int8_t decodeAgnssf00(unsigned char const *raw, size_t rawSize, BUF_t *bufs, int8_t numBufs)
{
  char const UBXAGNSSF[] = "ubxagnssf";
  uint16_t checksumRead = 0;
  uint8_t version = 0x00;                     // Version of the encoder
  size_t minRawSize = sizeof(checksumRead)    // Space for checksum
                      + strlen(UBXAGNSSF)     // Space for file id string
                      + sizeof(version)       // Space for version number
                      + sizeof(int8_t)        // Space for no. of entries
                      + 1 * sizeof(uint32_t); // Space for one table of entry

  // None of the arguments should break the basic rules
  if (!raw || !bufs || numBufs <= 0 || rawSize > (size_t)SSIZE_MAX || rawSize < minRawSize)
    return HELPER_ERR_INVALID_ARGUMENT;

  // Check if the file identifier can be found in the raw data
  size_t offset = 0;
  if (memcmp(raw + offset, UBXAGNSSF, strlen(UBXAGNSSF)))
    return HELPER_ERR_UNKNOWN_FORMAT;

  offset += strlen(UBXAGNSSF);

  // Make sure the ckecksum is correct
  uint16_t checksumCalc = calculateChecksum(raw, rawSize - sizeof(checksumCalc));
  memcpy(&checksumRead, &raw[rawSize - sizeof(checksumRead)], sizeof(checksumRead));

  if (checksumCalc != checksumRead)
    return HELPER_ERR_CHECKSUM_UBX;

  // Check the encoder version information.
  // This decoder only understands that one version

  if (memcmp(raw + offset, &version, sizeof(version)))
    return HELPER_ERR_UNSUPPORTED_VER;

  offset += sizeof(version);

  // Copy the number of tables stored in the buffer
  // to the numBufsFound variable
  int8_t numBufsFound = 0;
  memcpy(&numBufsFound, raw + offset, sizeof(numBufsFound));
  offset += sizeof(numBufsFound);

  // The one mandatory one was already calculated in at the beginning
  minRawSize += (numBufsFound - 1) * sizeof(uint32_t);

  // The number of entries in the file should not exceed the
  // array passed to this function, nor should it be an invalid
  // value. The calculated rawSize should also be big enough
  // to store at least the number of tables here
  if (numBufsFound < 0 || rawSize < minRawSize || numBufsFound > numBufs)
    return HELPER_ERR_INVALID_ENTRY_NUM;

  // Copy the table of entries from the raw file
  uint32_t entryTable[0x7F];
  memcpy(entryTable, raw + offset, numBufsFound * sizeof(uint32_t));
  offset += numBufsFound * sizeof(uint32_t);

  // The total size of all blocks combined
  // should not be bigger than the raw data
  // provided including overhead
  for (int8_t i = 0; i < numBufsFound; ++i)
  {
    // No block should be bigger than 1023 KiB
    if (entryTable[i] > 0xFFFFF)
      return HELPER_ERR_ENTRY_TOO_LONG;

    minRawSize += entryTable[i];
  }

  if (rawSize < minRawSize)
    return HELPER_ERR_ARG_TOO_SHORT;

  // Allocate the buffers for all the BUF_t provided,
  // copy the buffer to the newly allocated memory
  // and clean up if something goes wrong
  bool abortLoop = false;
  unsigned char *tmpBufs[0x7F]{};
  //lint -e{676} // numBufsFound is checked for being negative above.
  // It has not changed since. The corresponding lint
  // message is incorrect.
  for (int8_t i = 0; !abortLoop && i < numBufsFound; ++i)
  {
    // If a value in the entry table is zero,
    // just set the pointer to NULL as well,
    // otherwise allocate memory and copy
    // the contents
    if (entryTable[i] == 0)
    {
      tmpBufs[i] = NULL;
    }
    else
    {
      tmpBufs[i] = (unsigned char *)malloc(entryTable[i]);
      if (tmpBufs[i])
      {
        // Copy the data to the newly allocated
        // memory.
        memcpy(tmpBufs[i], raw + offset, entryTable[i]);
        offset += entryTable[i];
      }
      else
      {
        // Clean up everything if memory could
        // not be allocated
        for (int8_t j = 0; j < i; ++j)
        {
          free(tmpBufs[j]);
        }

        abortLoop = true;
      }
    }
  }

  // If everything went well copy the pointers
  // to the newly allocated data to the actual
  // buffers
  int8_t result = HELPER_ERR_OUT_OF_MEMORY;
  if (!abortLoop)
  {
    for (int8_t i = 0; i < numBufsFound; ++i)
    {
      bufs[i].p = tmpBufs[i];
      bufs[i].i = entryTable[i];
    }
    result = numBufsFound;
  }

  return result;
}

/*******************************************************************************
 * BUFFER HELPERS
 *******************************************************************************/

/*! Put the date into a buffer, this function checks if the data is the same as pervious data.
    If not it allocate a new buffer and copies the content of the new data. the it repalces the
    old data with the new one and frees the memory of the old data.

    \param pBuf the buffer to replace
    \param pMsg the pointer to the data to be stored
    \param iMsg the size of the data to be stored
    \return true if sucessfull, false if failed ot data was equal to already stored data
*/
bool replaceBuf(BUF_t *pBuf, const unsigned char *pMsg, unsigned int iMsg)
{
  if (!pBuf || !pMsg || !iMsg)
    return false;

  bool result = false;
  if (pBuf->p == NULL || pBuf->i != iMsg || memcmp(pBuf->p, pMsg, iMsg) != 0)
  {

    unsigned char *p = (unsigned char *)malloc(iMsg);
    if (p)
    {
      memcpy(p, pMsg, iMsg);

      if (pBuf->p != NULL)
        free(pBuf->p);

      pBuf->p = p;
      pBuf->i = iMsg;
      result = true;
    }
  }

  return result;
}

/*! Function to free the contents of a BUF_t

    \param pBuf       : The buffer to be freed.
    \return             true on success, false on failure
*/
bool freeBuf(BUF_t *pBuf)
{
  bool result = false;
  if (pBuf)
  {
    free(pBuf->p);
    pBuf->p = NULL;
    pBuf->i = 0;
    result = true;
  }

  return result;
}

/* Helper to free an array of buffers

    \param pBufs      : The buffers to be freed.
    \param iBufs      : The number of buffers in pBufs
    \return             true on success, false on failure
*/
bool freeBufs(BUF_t *pBufs, size_t iBufs)
{
  bool result = false;
  if (pBufs)
  {
    for (size_t i = 0; i < iBufs; ++i)
    {
      freeBuf(pBufs + i);
    }

    result = true;
  }
  return result;
}

/*! Write a UBX message to the receiver with a specific class/message id and for a certain payload.

    \param buf      Must not be NULL. Will contain the result
    \param classID  the class Id of the message to be sent
    \param msgID    the message Id of the message to be sent
    \param pData0   the pointer to the first part of the payload to be sent (can be NULL)
    \param iData0   the size of the first part of the payload to be sent (can be 0)
    \param pData1   the pointer to the second part of the payload to be sent (can be NULL)
    \param iData1   the size of the second part of the payload to be sent (can be 0)
    \param pData2   the pointer to the third part of the payload to be sent (can be NULL)
    \param iData2   the size of the third part of the payload to be sent (can be 0)
    \param pData3   the pointer to the fourth part of the payload to be sent (can be NULL)
    \param iData3   the size of the fourth part of the payload to be sent (can be 0)
    \param pData4   the pointer to the fifth part of the payload to be sent (can be NULL)
    \param iData4   the size of the fifth part of the payload to be sent (can be 0)
    \return         The number of bytes in buf if successful,
                    a negative numeber otherwise
*/
ssize_t createUbx(unsigned char **buf,
                  unsigned char classID,
                  unsigned char msgID,
                  const void *pData0,
                  size_t iData0,
                  const void *pData1,
                  size_t iData1,
                  const void *pData2,
                  size_t iData2,
                  const void *pData3,
                  size_t iData3,
                  const void *pData4,
                  size_t iData4)
{
  if ((!pData0 && !pData1) && (iData0 > 0 || iData1 > 0))
    return -1;

  ssize_t result = -1;
  BUFC_t d0;
  d0.p = (unsigned char const *)pData0;
  d0.i = iData0;

  BUFC_t d1;
  d1.p = (unsigned char const *)pData1;
  d1.i = iData1;

  BUFC_t d2;
  d2.p = (unsigned char const *)pData2;
  d2.i = iData2;

  BUFC_t d3;
  d3.p = (unsigned char const *)pData3;
  d3.i = iData3;

  BUFC_t d4;
  d4.p = (unsigned char const *)pData4;
  d4.i = iData4;

  std::list<BUFC_t> l;

  l.push_back(d0);
  l.push_back(d1);
  l.push_back(d2);
  l.push_back(d3);
  l.push_back(d4);
  result = createUbx(buf, classID, msgID, l);

  return result;
}

/*! Write a UBX message to the receiver with a specific class/message id and for a certain payload.

    \param buf      Must not be NULL. Will contain the result
    \param classID  the class Id of the message to be sent
    \param msgID    the message Id of the message to be sent
    \param payload  List of buffers containing payload
    \return         The number of bytes in buf if successful,
                    a negative numeber otherwise
*/
ssize_t createUbx(unsigned char **buf,
                  unsigned char classID,
                  unsigned char msgID,
                  std::list<BUFC_t> payload)
{
  const int SYNC_SIZE = 2;
  const int ID_SIZE = 2;
  const int LENGTH_SIZE = 2;
  const int HEADER_SIZE = SYNC_SIZE + ID_SIZE + LENGTH_SIZE;
  const int CRC_SIZE = 2;
  ssize_t sizeBuffer = 0;

  // Calculate payload buffer needed and make sure
  // all entries in the list are legal
  for (auto i : payload)
  {
    if (sizeBuffer >= 0)
    {
      if ((i.p && i.i) || (!i.p && !i.i))
        sizeBuffer += i.i;
      else
        sizeBuffer = -1;
    }
  }

  if (sizeBuffer < 0)
    return -1;

  if ((size_t)sizeBuffer > (size_t)(SSIZE_MAX - CRC_SIZE - HEADER_SIZE))
    return -1;

  // Add header and checksum size to buffer calculations
  sizeBuffer += CRC_SIZE + HEADER_SIZE;

  // Allocate the memory needed to store the message
  unsigned char *tmpBuf = (unsigned char *)malloc(sizeBuffer);

  if (!tmpBuf)
    return -1;

  // Create header
  size_t ubxPayLength = sizeBuffer - CRC_SIZE - HEADER_SIZE;
  tmpBuf[0] = 0xB5;
  tmpBuf[1] = 'b';
  tmpBuf[2] = classID;
  tmpBuf[3] = msgID;
  tmpBuf[4] = (uint8_t)ubxPayLength;
  tmpBuf[5] = (uint8_t)(ubxPayLength >> 8);

  size_t emptyStart = HEADER_SIZE;
  // Copy the data from the list to the buffer
  for (auto i : payload)
  {
    if (i.p && i.i)
    {
      memcpy(tmpBuf + emptyStart, i.p, i.i);
      emptyStart += i.i;
    }
  }

  // Calcualte Checksum
  uint16_t checksum = calculateChecksum(&tmpBuf[SYNC_SIZE], sizeBuffer - ID_SIZE - CRC_SIZE);
  tmpBuf[emptyStart] = (uint8_t)(checksum >> 8);
  ++emptyStart;
  tmpBuf[emptyStart] = (uint8_t)checksum;
  ++emptyStart;

  // Sanity check
  assert(emptyStart == (size_t)sizeBuffer);
  assert(isUbxMessage(tmpBuf, sizeBuffer, false));

  *buf = tmpBuf;

  return sizeBuffer;
}

/*! Converts a current time, which is relative to the last roll-over to a
    GPS time relative to the GPS epoch by using the earliest sane time as
    reference. This function can also be used to compensate potential roll-overs
    in GPS time relative to the GPS epoch.
    \warning If roll-over information is not available in the provided time,
    this function will start to create wrong results approximatly 20 years after
    the earliest sane time \ref SRC_CREAT_TIME_SEC

	\param gpsTime : Pointer to the time that should be recalculated
*/
void relToAbsGpsTime(GPS_TIME_t *gpsTime)
{
  if (gpsTime)
  {
    // This must always return true, otherwise there is
    // something wrong with the earlist time set
    GPS_TIME_t earliestSaneTime;
    bool gpsTimeRes = getEarliestSaneGpsTime(&earliestSaneTime);
    assertTrue(gpsTimeRes);
    if (!gpsTimeRes)
    {
      //this should not have happened, zero everything to be safe
      earliestSaneTime = {};
    }

    // Normalize the time
    normalizeTime(gpsTime);

    // Is this a time relative to the GPS time epoch or
    // relative to the last roll-over? If relative to last
    // roll-over (< GPS_WEEKS_BEF_RO), use roll-over of earliest
    // sane time to make absolute
    if (gpsTime->wno + gpsTime->ro * GPS_WEEKS_BEF_RO < GPS_WEEKS_BEF_RO)
      gpsTime->ro = earliestSaneTime.ro;

    // If gpsTime is earlier than the earliest sane time,
    // assume a rollover took place
    if (timecmp(gpsTime, &earliestSaneTime) == -1)
      ++gpsTime->ro;
  }
}

/*! Make sure the GPS time provided is normalized
    \param gpsTime   : The time that has to be normalized
*/
void normalizeTime(GPS_TIME_t *gpsTime)
{
  if (gpsTime)
  {
    // Normalize tow part
    normalizeTime(&gpsTime->tow);

    // If there has been a time of week overflow in, compensate
    // with the week number
    while (gpsTime->tow.tv_sec >= SEC_PER_WEEK)
    {
      ++gpsTime->wno;
      gpsTime->tow.tv_sec -= SEC_PER_WEEK;
    }

    while (gpsTime->tow.tv_sec < 0)
    {
      --gpsTime->wno;
      gpsTime->tow.tv_sec += SEC_PER_WEEK;
    }

    // Compensate week number roll-overs in one or the
    // other direction
    while (gpsTime->wno < 0)
    {
      --gpsTime->ro;
      gpsTime->wno += GPS_WEEKS_BEF_RO;
    }

    while (gpsTime->wno > GPS_WEEKS_BEF_RO)
    {
      ++gpsTime->ro;
      gpsTime->wno -= GPS_WEEKS_BEF_RO;
    }
  }
}

/*! Make sure the nanosecond part is >=0 and <= 999999999
    \param unTime   : The time that has to be normalized
*/
void normalizeTime(struct timespec *unTime)
{
  if (unTime)
  {
    // Add whole seconds to the seconds part and subtract
    // it's equivalent in nanosecons from the nanoseconds part
    // until the nanoseconds part is smaller than 1 second
    while (unTime->tv_nsec >= NSEC_PER_SEC)
    {
      ++unTime->tv_sec;
      unTime->tv_nsec -= NSEC_PER_SEC;
    }

    // Remove seconds from the seconds part and add
    // it's equivalent in annoseconds to the nanoseconds part
    // until the nanoseconds part is greater or equal 0
    while (unTime->tv_nsec < 0)
    {
      --unTime->tv_sec;
      unTime->tv_nsec += NSEC_PER_SEC;
    }

    assert(unTime->tv_nsec >= 0 && unTime->tv_nsec < NSEC_PER_SEC);
  }
}

/*! The timecmp() compares the provided times and determines which one is
    earlier in time. The function returns an integer less than, equal to,
    or greater than zero if the first time is found, respectively, to be
    less than, to match, or be greater than the second time provided. If
    one or both pointers provided are NULL, 0 is returned.
    \param time1      : The pointer to the first time to compare
    \param time2      : The pointer to the second time to compare
    \return           : -1 if the first time is earlier than the second
                         0 if the times are equal or one of them is NULL
                         1 if the first time is later than the second
*/
int timecmp(GPS_TIME_t const *time1, GPS_TIME_t const *time2)
{
  if (!time1 || !time2)
    return 0;

  int result = 0;
  GPS_TIME_t t1, t2;
  memcpy(&t1, time1, sizeof(t1));
  memcpy(&t2, time2, sizeof(t2));
  normalizeTime(&t1);
  normalizeTime(&t2);

  if (t1.ro == t2.ro)
  {
    // The second values are the same, it is required
    // to compare the nanoseconds value.
    if (t1.wno > t2.wno)
      result = 1;
    else if (t1.wno < t2.wno)
      result = -1;
    else // Now, time of week has to be taken into consideration
      result = timecmp(&t1.tow, &t2.tow);
  }
  else if (t1.ro > t2.ro)
  {
    result = 1;
  }
  else // t1.ro < t2.ro
  {
    result = -1;
  }

  return result;
}

/*! The timecmp() compares the provided times and determines which one is
    earlier in time. The function returns an integer less than, equal to,
    or greater than zero if the first time is found, respectively, to be
    less than, to match, or be greater than the second time provided. If
    one or both pointers provided are NULL, 0 is returned.
    \param time1      : The pointer to the first time to compare
    \param time2      : The pointer to the second time to compare
    \return           : -1 if the first time is earlier than the second
                         0 if the times are equal or one of them is NULL
                         1 if the first time is later than the second
*/
int timecmp(struct timespec const *time1, struct timespec const *time2)
{
  if (!time1 || !time2)
    return 0;

  int result = 0;
  struct timespec t1, t2;
  memcpy(&t1, time1, sizeof(t1));
  memcpy(&t2, time2, sizeof(t2));
  normalizeTime(&t1);
  normalizeTime(&t2);

  if (t1.tv_sec == t2.tv_sec)
  {
    // The second values are the same, it is required
    // to compare the nanoseconds value.
    if (t1.tv_nsec > t2.tv_nsec)
      result = 1;
    else if (t1.tv_nsec < t2.tv_nsec)
      result = -1;
  }
  else if (t1.tv_sec > t2.tv_sec)
  {
    result = 1;
  }
  else // t1.tv_sec < t2.tv_sec
  {
    result = -1;
  }

  return result;
}

time_t getEarliestSaneTime(struct timespec *earliestTime /* = NULL */)
{
  if (earliestTime)
  {
    earliestTime->tv_sec = SRC_CREAT_TIME_SEC;
    earliestTime->tv_nsec = 0;
  }

  return SRC_CREAT_TIME_SEC;
}

bool getEarliestSaneGpsTime(GPS_TIME_t *earliestTime)
{
  if (!earliestTime)
    return false;

  // Get the source creation time as Unix time
  struct timespec unixTime;
  getEarliestSaneTime(&unixTime);

  // This must always return true, otherwise there is
  // something wrong with the earlist time
  bool result = unixToGpsTime(&unixTime, earliestTime, false);
  assert(result);

  return result;
}

time_t getEarliestSaneGpsTime(struct timespec *earliestTime /* = NULL */)
{
  // Get the source creation time as Unix time
  struct timespec unixTime;
  struct timespec gpsTime;

  getEarliestSaneTime(&unixTime);

  // This must always return true, otherwise there is
  // something wrong with the earlist time
  bool u2gRes = unixToGpsTime(&unixTime, &gpsTime, false);
  assertTrue(u2gRes);

  //this should never happen, return something "safe" in any case.
  if (!u2gRes)
    return LONG_MAX;

  if (earliestTime)
    memcpy(earliestTime, &gpsTime, sizeof(*earliestTime));

  return gpsTime.tv_sec;
}

bool gpsToUnixTime(GPS_TIME_t const *gpsTime,
                   struct timespec *unixTime,
                   bool unixNeedsLeap /* = false */)
{
  if (!gpsTime)
    return false;

  GPS_TIME_t normGpsTime;
  memcpy(&normGpsTime, gpsTime, sizeof(normGpsTime));
  normalizeTime(&normGpsTime);

  // If the normalisation resulted in a negative value in
  // any parts, this can not be converted properly
  if (normGpsTime.ro < 0 || normGpsTime.wno < 0 || normGpsTime.tow.tv_sec < 0 ||
      normGpsTime.tow.tv_nsec < 0)
    return false;

  // Convert the GPS time in roll-over, week number, time of week
  // representation to seconds since GPS Epoch
  struct timespec gpsTimeSec;
  memcpy(&gpsTimeSec, &normGpsTime.tow, sizeof(gpsTimeSec));

  gpsTimeSec.tv_sec += (normGpsTime.ro * GPS_WEEKS_BEF_RO + normGpsTime.wno) * SEC_PER_WEEK;

  // Let the other gpsToUnixTime() function do the actual work
  return gpsToUnixTime(&gpsTimeSec, unixTime, unixNeedsLeap);
}

bool gpsToUnixTime(struct timespec const *gpsTime,
                   struct timespec *unixTime,
                   bool unixNeedsLeap /* = false */)
{
  if (!unixTime || !gpsTime)
    return false;

  struct timespec normGpsTime;
  time_t gpsEpochStartSec = GPS_EPOCH_SEC;

  // Some leap seconds were introduced before the
  // GPS epoch. Compensate the GPS Epoch accordingly for the
  // eventual subtraction from the Unix time later on.
  // Otherwise the Unix time will be substracted
  // to many seconds. Only the ones since GPS
  // Epoch must be removed.
  if (unixNeedsLeap)
    gpsEpochStartSec = addLeapSec(GPS_EPOCH_SEC);

  memcpy(&normGpsTime, gpsTime, sizeof(normGpsTime));

  normalizeTime(&normGpsTime);

  // Add the GPS epoch to get Unix time in seconds and
  // write it to the resulting timespec structure
  unixTime->tv_sec = normGpsTime.tv_sec + gpsEpochStartSec;
  unixTime->tv_nsec = normGpsTime.tv_nsec;

  // Compensate for leap seconds (GPS does not know leap seconds)
  // if unix time needs the leap seconds deducted.
  if (unixNeedsLeap)
    deductLeapSec(unixTime);

  return true;
}

bool unixToGpsTime(struct timespec const *unixTime,
                   GPS_TIME_t *gpsTime,
                   bool unixHasLeap /* = false */)
{
  if (!gpsTime)
    return false;

  if (!unixToGpsTime(unixTime, &gpsTime->tow, unixHasLeap))
    return false;

  // Calculate week number and roll over
  gpsTime->wno = gpsTime->tow.tv_sec / SEC_PER_WEEK;
  gpsTime->ro = gpsTime->wno / GPS_WEEKS_BEF_RO;
  gpsTime->wno %= GPS_WEEKS_BEF_RO;

  // Calculate time of week
  gpsTime->tow.tv_sec %= SEC_PER_WEEK;

  return true;
}

bool unixToGpsTime(struct timespec const *unixTime,
                   struct timespec *gpsTime,
                   bool unixHasLeap /* = false */)
{
  if (!unixTime || !gpsTime)
    return false;

  bool result = false;
  struct timespec normUnixTime;
  time_t gpsEpochStartSec = GPS_EPOCH_SEC;

  memcpy(&normUnixTime, unixTime, sizeof(normUnixTime));

  // Compensate for leap seconds (GPS does not know leap seconds),
  // if unix time has the leap seconds deducted. addLeapSec does
  // does time normalization. If not executed, do it manually
  if (unixHasLeap)
  {
    addLeapSec(&normUnixTime);

    // Some leap seconds were introduced before the
    // GPS epoch. Compensate the GPS Epoch accordingly for the
    // eventual subtraction from the Unix time later on
    gpsEpochStartSec = addLeapSec(GPS_EPOCH_SEC);
    assert(gpsEpochStartSec != (time_t)-1);
  }
  else
  {
    normalizeTime(&normUnixTime);
  }

  if (normUnixTime.tv_sec > gpsEpochStartSec)
  {
    // Deduct the GPS epoch to get GPS time in seconds and
    // write it to the resulting timespec structure
    gpsTime->tv_sec = normUnixTime.tv_sec - gpsEpochStartSec;
    gpsTime->tv_nsec = normUnixTime.tv_nsec;
    result = true;
  }

  return result;
}

static bool modLeapSecond(struct timespec *unixTime, bool deduct)
{
  if (!unixTime)
    return false;

  size_t numOfLeapSec = 0;

  normalizeTime(unixTime);

  // Count the number of leap seconds passed since the specified
  // unix date
  for (; numOfLeapSec < UNIX_LEAP_SEC; ++numOfLeapSec)
  {
    if (LEAP_SEC[numOfLeapSec] > unixTime->tv_sec)
      break;
  }

  if (deduct)
    unixTime->tv_sec -= numOfLeapSec;
  else
    unixTime->tv_sec += numOfLeapSec;

  return true;
}

time_t deductLeapSec(time_t unixTime)
{
  struct timespec uT;
  time_t result = (time_t)-1;

  uT.tv_sec = unixTime;
  uT.tv_nsec = 0;

  if (deductLeapSec(&uT))
    result = uT.tv_sec;

  return result;
}

bool deductLeapSec(struct timespec *unixTime) { return modLeapSecond(unixTime, true); }

bool deductLeapSec(ACCTIME_t *accTime)
{
  if (!accTime)
    return false;

  bool result = false;

  if (!accTime->leapSeconds)
  {
    result = deductLeapSec(&accTime->time);
    accTime->leapSeconds = result;
  }

  return result;
}

time_t addLeapSec(time_t unixTime)
{
  struct timespec uT;
  time_t result = (time_t)-1;

  uT.tv_sec = unixTime;
  uT.tv_nsec = 0;

  if (addLeapSec(&uT))
    result = uT.tv_sec;

  return result;
}

bool addLeapSec(struct timespec *unixTime) { return modLeapSecond(unixTime, false); }

bool addLeapSec(ACCTIME_t *accTime)
{
  if (!accTime)
    return false;

  bool result = false;

  if (accTime->leapSeconds)
  {
    result = addLeapSec(&accTime->time);
    accTime->leapSeconds = !result;
  }

  return result;
}

bool getMonotonicCounter(struct timespec *monTime)
{
  if (!monTime)
    return false;

  bool result = false;

  if (!clock_gettime(CLOCK_MONOTONIC, monTime))
    result = true;

  return result;
}

bool isValidAccTime(ACCTIME_t const *accTime)
{
  if (!accTime || !accTime->valid)
    return false;

  bool result = false;
  struct timespec earliestPossTime;
  getEarliestSaneTime(&earliestPossTime);

  if (timecmp(&accTime->time, &earliestPossTime) >= 0) // valid time?
    result = true;

  return result;
}

bool isValidPos(POS_t const *pos)
{
  if (!pos || !pos->valid)
    return false;

  bool result = true;

  if (pos->latDeg >= MIN_LAT_DEG && pos->latDeg <= MAX_LAT_DEG && pos->lonDeg >= MIN_LON_DEG &&
      pos->lonDeg <= MAX_LON_DEG && pos->posAccCm <= MAX_POS_INACC)
    result = true;

  return result;
}

bool extractMgaIniTimeUtc(MGA_INI_TIME_UTC_t const *data, ACCTIME_t *accTime)
{
  if (!data || !accTime)
    return false;

  bool result = false;
  // Inject the server time to the time handler
  struct tm serverTime
  {
  };
  unsigned const char *iniPay = data->pay;
  serverTime.tm_year = ((iniPay[5] << 8) + iniPay[4]) - 1900;
  serverTime.tm_mon = iniPay[6] - 1;
  serverTime.tm_mday = iniPay[7];
  serverTime.tm_hour = iniPay[8];
  serverTime.tm_min = iniPay[9];
  serverTime.tm_sec = iniPay[10];

  // Extract the required information and fill into accTimeNow
  ACCTIME_t tmpAccTime;
  tmpAccTime.valid = true;
  tmpAccTime.leapSeconds = true; // We assume we always got leapseconds
  tmpAccTime.time.tv_sec = timegm(&serverTime);
  tmpAccTime.time.tv_nsec =
    (iniPay[15] << 24) + (iniPay[14] << 16) + (iniPay[13] << 8) + iniPay[12];
  tmpAccTime.acc.tv_sec = (iniPay[17] << 8) + iniPay[16];
  tmpAccTime.acc.tv_nsec = (iniPay[23] << 24) + (iniPay[14] << 22) + (iniPay[21] << 8) + iniPay[20];

  // Check if the resulting time is valid and
  // if yes copy it to the destination
  if (isValidAccTime(&tmpAccTime))
  {
    memcpy(accTime, &tmpAccTime, sizeof(*accTime));
    result = true;
  }

  return result;
}

bool createMgaIniTimeUtc(MGA_INI_TIME_UTC_t *data, ACCTIME_t const *accTime)
{
  if (!data || !accTime || !isValidAccTime(accTime))
    return false;

  bool result = false;
  struct tm brokenDown;

  if (gmtime_r(&accTime->time.tv_sec, &brokenDown))
  {
    // Adjust year and month which
    // both start with from a different
    // offset from UTC
    brokenDown.tm_year += 1900;
    brokenDown.tm_mon += 1;

    // Set the data correctly
    std::fill(std::begin(data->pay), std::end(data->pay), 0);
    // Type of MGA-INI message
    data->pay[0] = (U1)0x10;

    // Set Leap second unknown
    data->pay[3] = (U1)0x80;

    // Set time
    data->pay[4] = (U1)brokenDown.tm_year & 0xFF;
    data->pay[5] = (U1)(brokenDown.tm_year >> 8) & 0xFF;
    data->pay[6] = (U1)brokenDown.tm_mon;
    data->pay[7] = (U1)brokenDown.tm_mday;
    data->pay[8] = (U1)brokenDown.tm_hour;
    data->pay[9] = (U1)brokenDown.tm_min;
    data->pay[10] = (U1)brokenDown.tm_sec;
    data->pay[12] = (U1)accTime->time.tv_nsec & 0xFF;
    data->pay[13] = (U1)(accTime->time.tv_nsec >> 8) & 0xFF;
    data->pay[14] = (U1)(accTime->time.tv_nsec >> 16) & 0xFF;
    data->pay[15] = (U1)(accTime->time.tv_nsec >> 24) & 0xFF;

    // Set accuracy
    data->pay[16] = (U1)accTime->acc.tv_sec & 0xFF;
    data->pay[17] = (U1)(accTime->acc.tv_sec >> 8) & 0xFF;
    data->pay[20] = (U1)accTime->acc.tv_nsec & 0xFF;
    data->pay[21] = (U1)(accTime->acc.tv_nsec >> 8) & 0xFF;
    data->pay[22] = (U1)(accTime->acc.tv_nsec >> 16) & 0xFF;
    data->pay[23] = (U1)(accTime->acc.tv_nsec >> 24) & 0xFF;

    result = true;
  }

  return result;
}

bool createMgaIniPos(MGA_INI_POS_t *data, POS_t const *pos)
{
  if (!data || !pos || !isValidPos(pos))
    return false;

  // Set the data correctly
  std::fill(std::begin(data->pay), std::end(data->pay), 0);

  // Type of MGA-INI message (In this case MGA-INI-POS_LLH)
  data->pay[0] = (U1)0x01;

  // Set latitude
  data->pay[4] = (U1)pos->latDeg & 0xFF;
  data->pay[5] = (U1)(pos->latDeg >> 8) & 0xFF;
  data->pay[6] = (U1)(pos->latDeg >> 16) & 0xFF;
  data->pay[7] = (U1)(pos->latDeg >> 24) & 0xFF;

  // Set longitude
  data->pay[8] = (U1)pos->lonDeg & 0xFF;
  data->pay[9] = (U1)(pos->lonDeg >> 8) & 0xFF;
  data->pay[10] = (U1)(pos->lonDeg >> 16) & 0xFF;
  data->pay[11] = (U1)(pos->lonDeg >> 24) & 0xFF;

  // Set Altitude (unknown)
  data->pay[15] = (U1)0x80;

  // Set Accuracy
  data->pay[16] = (U1)pos->posAccCm & 0xFF;
  data->pay[17] = (U1)(pos->posAccCm >> 8) & 0xFF;
  data->pay[18] = (U1)(pos->posAccCm >> 16) & 0xFF;
  data->pay[19] = (U1)(pos->posAccCm >> 24) & 0xFF;

  return true;
}

/*! Extract a UBX-AID-INI message
    \param pay        : The payload which will be created on success.
                        Must not be NULL
    \param accTime    : The struct to which the payload is extracted to.
                        May be NULL if pos is not.
    \param pos        : The struct to which the payload is extracted to.
                        May be NULL if time is not.
    \return             true on success. false otherwise.
*/
bool extractUbxAidIni(GPS_UBX_AID_INI_U5__t const *pay, ACCTIME_t *accTime, POS_t *pos /* = NULL */)
{
  if (!pay || !(accTime || pos))
    return false;

  bool result = false;

  // extract position
  // Position can only be extracted in degrees
  if (pos && pay->flags & GPS_UBX_AID_INI_U5__FLAGS_POS_MASK & GPS_UBX_AID_INI_U5__FLAGS_LLA_MASK)
  {
    pos->latDeg = pay->ecefXOrLat;
    pos->lonDeg = pay->ecefYOrLon;
    pos->posAccCm = pay->posAcc;
    pos->valid = true;
    result = true;
  }

  // extract time
  if (accTime && pay->flags & GPS_UBX_AID_INI_U5__FLAGS_TIME_MASK)
  {
    const uint64_t oneWeekMs = MSEC_PER_SEC * SEC_PER_WEEK;
    uint64_t leapSecMs = MSEC_PER_SEC * GPS_LEAP_SEC;
    uint64_t timeGpsMs = ((uint64_t)pay->wn) * oneWeekMs + (uint64_t)pay->tow - leapSecMs;
    // convert to Unix time
    timeGpsMs += (int64_t)GPS_EPOCH_SEC * MSEC_PER_SEC;

    accTime->time.tv_sec = (time_t)((timeGpsMs) / MSEC_PER_SEC);
    accTime->time.tv_nsec = ((long)(timeGpsMs % MSEC_PER_SEC)) * NSEC_PER_MSEC + pay->towNs;
    normalizeTime(&accTime->time);

    uint64_t accNs = ((uint64_t)pay->tAccMs) * NSEC_PER_MSEC + pay->tAccNs;
    accTime->acc.tv_sec = (time_t)(accNs / NSEC_PER_SEC);
    accTime->acc.tv_nsec = (long)(accNs % NSEC_PER_SEC);
    accTime->valid = true;
    accTime->leapSeconds = true;
    normalizeTime(&accTime->acc);
    result = true;
  }

  return result;
}

/*! Create a UBX-AID-INI message
    \param pay        : The payload which will be created on success.
                        Must not be NULL
    \param accTime    : The time which is used to create the payload.
                        The accuracy must be set to a valid value as
                        well as the valid flag must be set, for the data
                        to be used. May be NULL if pos is not.
    \param pos        : The position which is used to create the payload.
                        The accuracy must be set to a valid value
                        as well as the valid flag must be set, for the
                        the data to be used. May be NULL if time is not.
    \return             true on success. false otherwise.
*/
bool createUbxAidIni(GPS_UBX_AID_INI_U5__t *pay,
                     ACCTIME_t const *accTime,
                     POS_t const *pos /* = NULL */)
{
  if (!pay ||
      (!(isValidAccTime(accTime) && (accTime->acc.tv_sec || accTime->acc.tv_nsec)) // valid time?
       &&
       !(isValidPos(pos) && pos->posAccCm))) // valid position?
    return false;

  GPS_UBX_AID_INI_U5__t tmpPay{};
  bool result = false;

  // position aiding
  if (pos)
  {
    tmpPay.ecefXOrLat = pos->latDeg;
    tmpPay.ecefYOrLon = pos->lonDeg;
    tmpPay.posAcc = pos->posAccCm;
    tmpPay.flags |= GPS_UBX_AID_INI_U5__FLAGS_POS_MASK | GPS_UBX_AID_INI_U5__FLAGS_LLA_MASK |
                    GPS_UBX_AID_INI_U5__FLAGS_ALTINV_MASK;
    result = true;
  }

  // time aiding
  //lint -e{571}
  if (accTime)
  {
    const uint64_t oneWeekMs = MSEC_PER_SEC * 60 * 60 * 24 * 7;
    uint64_t leapSecMs = 0;

    // If leap seconds are inlcuded in the provided time, they
    // have to be added again for GPS time which does not know
    // leap seconds and is thus ahead of UTC
    if (accTime->leapSeconds)
      leapSecMs = MSEC_PER_SEC * GPS_LEAP_SEC;
    uint64_t timeGpsMs = (uint64_t)accTime->time.tv_sec * (uint64_t)MSEC_PER_SEC +
                         (uint64_t)accTime->time.tv_nsec / (uint64_t)NSEC_PER_MSEC +
                         (uint64_t)leapSecMs;

    // Unix time could be older than GPS time. If this is the case don't
    // encode the time
    if (timeGpsMs >= (uint64_t)GPS_EPOCH_SEC * MSEC_PER_SEC)
    {
      // convert Unix time to GPS time
      //lint -e(571)
      timeGpsMs -= (uint64_t)GPS_EPOCH_SEC * MSEC_PER_SEC;
      uint64_t accNs =
        ((uint64_t)accTime->acc.tv_sec) * (uint64_t)NSEC_PER_SEC + (uint64_t)accTime->acc.tv_nsec;
      tmpPay.tAccMs = accNs / NSEC_PER_MSEC;
      tmpPay.tAccNs = accNs % NSEC_PER_MSEC;
      tmpPay.wn = (U2)(timeGpsMs / oneWeekMs);
      tmpPay.tow = (uint32_t)(timeGpsMs % oneWeekMs);
      tmpPay.towNs = accTime->time.tv_nsec % NSEC_PER_MSEC;
      tmpPay.flags |= GPS_UBX_AID_INI_U5__FLAGS_TIME_MASK;
      result = true;
    }
  }
  memcpy(pay, &tmpPay, sizeof(*pay));

  return result;
}
