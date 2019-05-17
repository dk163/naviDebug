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
    Declaration of functions used in the AGNSS part of the driver
*/

#pragma once
#include "helperTypes.h"
#include <list>

//! Return string to corresponding id
char const *gnssIdToString(unsigned int gnssId);

//! Return string to corresponding data type to string
char const *aidingDataTypeToString(unsigned int dataType);

//! Return string to corresponding MGA ini type to string
char const *mgaIniDataTypeToString(unsigned int iniType);

//! Return string to corresponding service type
char const *agnssServiceTypeToString(unsigned int iniType);

//! Return string to corresponding action type
char const *agnssActionToString(unsigned int iniType);

//! Will generate a string matching to the known UBX message class and id
char const *ubxIdsToString(unsigned int classId, unsigned int msgId);

//! The "UNKNOWN" string used if no matching translation can be found
char const *const STR_UNKNOWN = "UNKNOWN";

//! Log to the provided log function, if valid
int logTo(LOG_FUNC_p func, LOG_PRIO_t prio, char const *format, ...);

//! Verifies if the passed data contains a valid UBX Message data
ssize_t isUbxMessage(const unsigned char *pBuffer, size_t iSize, bool ignTrailCh = false);

//! Verifies the passed data contains only whitelisted UBX messages
ssize_t verifyUbxMsgsBlock(unsigned char const *buf,
                           size_t size,
                           UBX_MSG_TYPE const *allowed,
                           size_t allowed_num,
                           BUFC_t **msgs);

//! Checks if the provided UBX message is part of the whitelisted messages
bool isAllowedUbxMsg(unsigned char const *buf,
                     int size,
                     UBX_MSG_TYPE const *allowed,
                     size_t allowed_num,
                     bool enWildCards = false);

//! Checks if the data contains a valid ALP file and returns the header
int verifyAlpData(const unsigned char *pData, unsigned int iData, ALP_FILE_HEADER_t *header);

//! Read all available bytes on the file descriptor
ssize_t availRead(int fd, unsigned char **buf, size_t maxSize = 0);

//! Read data from a file descriptor with timeout end end sequence
ssize_t extRead(int fd,
                unsigned char *buf,
                size_t bufSize,
                unsigned char const *stopSeq,
                size_t stopSeqSize,
                int timeoutMs);

//! Calculate 8-bit Fletcher checksum
uint16_t
calculateChecksum(unsigned char const *const data, size_t size, uint16_t const *chk = NULL);

//! Encode an array of buffers into an Agnssf00 format
ssize_t
encodeAgnssf00(unsigned char **raw, BUF_t const *bufs, int8_t numBufs, LOG_FUNC_p logFunc = NULL);

//! Decode an Agnssf00 formatted memory location into an array of buffers
int8_t decodeAgnssf00(unsigned char const *raw, size_t rawSize, BUF_t *bufs, int8_t numBufs);

//! Replace the content of buf with pMsg
bool replaceBuf(BUF_t *buf, const unsigned char *pMsg, unsigned int iMsg);

//! Free a buffer
bool freeBuf(BUF_t *pBuf);

//! Free an array of buffers
bool freeBufs(BUF_t *pBufs, size_t iBufs);

//! Write a UBX message to the receiver
ssize_t createUbx(unsigned char **buf,
                  unsigned char classID,
                  unsigned char msgID,
                  const void *pData0 = NULL,
                  size_t iData0 = 0,
                  const void *pData1 = NULL,
                  size_t iData1 = 0,
                  const void *pData2 = NULL,
                  size_t iData2 = 0,
                  const void *pData3 = NULL,
                  size_t iData3 = 0,
                  const void *pData4 = NULL,
                  size_t iData4 = 0);

//! Write a UBX message to the receiver
ssize_t createUbx(unsigned char **buf,
                  unsigned char classID,
                  unsigned char msgID,
                  std::list<BUFC_t> payload);

//! Converts a current time relative to the last roll-over to absolute GPS time
void relToAbsGpsTime(GPS_TIME_t *gpsTime);

//! Normalize the GPS_TIME_t provided
void normalizeTime(GPS_TIME_t *unTime);

//! Make sure the nanosecond part is >=0 and <= 999999999
void normalizeTime(struct timespec *unTime);

//! Find the earlier time of the two provided times
int timecmp(GPS_TIME_t const *time1, GPS_TIME_t const *time2);

//! Find the earlier time of the two provided times
int timecmp(struct timespec const *time1, struct timespec const *time2);

//! Get the earliest sane time that the system clock could represent
time_t getEarliestSaneTime(struct timespec *earliestTime = NULL);

//! Get the earliest sane time that the system clock could represent as GPS time
bool getEarliestSaneGpsTime(GPS_TIME_t *earliestTime);

//! Get the earliest sane time that the system clock could represent as seconds since the GPS Epoch
time_t getEarliestSaneGpsTime(struct timespec *earliestTime = NULL);

//! Convert GPS time (roll-over, weeks numbers, time of week) to Unix time
bool gpsToUnixTime(GPS_TIME_t const *gpsTime,
                   struct timespec *unixTime,
                   bool unixNeedsLeap = false);

//! Convert GPS time (seconds and nanoseconds since GPS Epoch) to Unix time
bool gpsToUnixTime(struct timespec const *gpsTime,
                   struct timespec *unixTime,
                   bool unixNeedsLeap = false);

//! Convert Unix time (seconds and nanoseconds since Unix Epoch) to GPS time (weeks, rollovers, seconds and nanoseconds since GPS Epoch)
bool unixToGpsTime(struct timespec const *unixTime, GPS_TIME_t *gpsTime, bool unixHasLeap = false);

//! Convert Unix time (seconds and nanoseconds since Unix Epoch) to GPS time (seconds and nanoseconds since GPS Epoch)
bool unixToGpsTime(struct timespec const *unixTime,
                   struct timespec *gpsTime,
                   bool unixHasLeap = false);

//! Substract the leap seconds since the Unix epoch to compensate for earth rotation
time_t deductLeapSec(time_t unixTime);

//! Substract the leap seconds since the Unix epoch to compensate for earth rotation
bool deductLeapSec(struct timespec *unixTime);

//! Substract the leap seconds since the Unix epoch, if not done already, to compensaste for earth rotation
bool deductLeapSec(ACCTIME_t *accTime);

//! Add the leap seconds since the Unix epoch, to not compensate for earth rotation any more
time_t addLeapSec(time_t unixTime);

//! Add the leap seconds since the Unix epoch, to not compensate for earth rotation any more
bool addLeapSec(struct timespec *unixTime);

//! Add the leap seconds since the Unix epoch, if not done already, to not compensate for earth rotation any more
bool addLeapSec(ACCTIME_t *accTime);

//! Get a value from the monotonic time counter of the OS
bool getMonotonicCounter(struct timespec *monTime);

//! Check if the parameter is a valid \ref ACCTIME_t
bool isValidAccTime(ACCTIME_t const *accTime);

//! Check if the parameter is a valid \ref POS_t
bool isValidPos(POS_t const *pos);

//! Extract the time from the message payload
bool extractMgaIniTimeUtc(MGA_INI_TIME_UTC_t const *data, ACCTIME_t *accTime);

//! Create the payload for an MGA-INI-TIME_UTC message from the given time
bool createMgaIniTimeUtc(MGA_INI_TIME_UTC_t *data, ACCTIME_t const *accTime);

//! Create the payload for an MGA-INI-TIME_LLH message from the given position
bool createMgaIniPos(MGA_INI_POS_t *data, POS_t const *pos);

//! Extract a UBX-AID-INI message
bool extractUbxAidIni(GPS_UBX_AID_INI_U5__t const *pay, ACCTIME_t *accTime, POS_t *pos = NULL);

//! Create a UBX-AID-INI message
bool createUbxAidIni(GPS_UBX_AID_INI_U5__t *pay, ACCTIME_t const *accTime, POS_t const *pos = NULL);
