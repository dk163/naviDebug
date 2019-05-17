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
    Definition of types and constants used in the AGNSS part of the driver

    \warning
        Both \ref LEAP_SEC and \ref SRC_CREAT_TIME_SEC have to be updated
        latest after every GPS week-rollover or leap-second change!
*/

#ifndef HELPER_TYPES
#define HELPER_TYPES
#include "../../hal/ubx_messageDef.h"
#include <stddef.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>

// Compilation time information. Must be updated after leap-second introduction or GPS week-rollover!
//! Leap seconds between Unix Epoch 1970.01.01T00:00:00 and 2015.06.30T00:00:00
const time_t LEAP_SEC[] = {
  78710400,
  94608000 // 1972.06.30T00:00:00, 1972.12.31T00:00:00
  ,
  126144000 //                      1973.12.31T00:00:00
  ,
  157680000 //                      1974.12.31T00:00:00
  ,
  189216000 //                      1975.12.31T00:00:00
  ,
  220838400 //                      1976.12.31T00:00:00
  ,
  252374400 //                      1977.12.31T00:00:00
  ,
  283910400 //                      1978.12.31T00:00:00
  ,
  315446400 //                      1979.12.31T00:00:00
  ,
  362707200 // 1981.06.30T00:00:00  //FIRST IN GPS!
  ,
  394243200 // 1982.06.30T00:00:00
  ,
  425779200 // 1983.06.30T00:00:00
  ,
  488937600 // 1985.06.30T00:00:00
  ,
  567907200 //                      1987.12.31T00:00:00
  ,
  631065600 //                      1989.12.31T00:00:00
  ,
  662601600 //                      1990.12.31T00:00:00
  ,
  709862400 // 1992.06.30T00:00:00
  ,
  741398400 // 1993.06.30T00:00:00
  ,
  772934400 // 1994.06.30T00:00:00
  ,
  820368000 //                      1995.12.31T00:00:00
  ,
  883526400 // 1997.06.30T00:00:00
  ,
  915062400 //                      1998.12.31T00:00:00
  ,
  1135987200 //                      2005.12.31T00:00:00
  ,
  1230681600 //                      2008.12.31T00:00:00
  ,
  1341014400 // 2012.06.30T00:00:00
  ,
  1435622400 // 2015.06.30T00:00:00
  ,
  1483142400 //                      2016.12.31T00:00:00
};

#define UNIX_LEAP_SEC                                                                              \
  (sizeof(LEAP_SEC) /                                                                              \
   sizeof(LEAP_SEC[0])) //!< Number of Leap seconds since Unix Epoch 1970.01.01T00:00:00
#define SRC_CREAT_TIME_SEC                                                                         \
  1472688000LL //!< Earliest possible time this software might be existing at 2016-09-01T00:00:00+00:00

// GPS and UTC time related (conversion) constants
#define GPS_LEAP_SEC (UNIX_LEAP_SEC - 9) //!< GPS leap second since 1980.1.6T00:00:00
#define GPS_EPOCH_SEC                                                                              \
  315964800LL //!< GPS Epoch in seconds since the Unix Epoch (this is mkgmtime(1980,1,6,0,0,0,0))
#define GPS_WEEKS_BEF_RO 1024 //!< GPS weeks before rollover

// Time conversion constants
#define NSEC_PER_SEC 1000000000LL //!< Number of nanoseconds per second
#define NSEC_PER_MSEC 1000000LL   //!< Number of nanoseconds per millisecond
#define USEC_PER_SEC 1000000LL    //!< Number of microseconds per second
#define MSEC_PER_SEC 1000LL       //!< Number of milliseconds per Second
#define SEC_PER_WEEK 604800LL     //!< Number of seconds per week (60 * 60 * 24 * 7)

// Location related constants
#define MAX_POS_INACC 600000000LL //!< The maximum position inaccuracy acceptable 6000km
#define MIN_LAT_DEG -900000000LL  //!< Minimum Latitude:   -90°
#define MAX_LAT_DEG 900000000LL   //!< Maximum Latitude:    90°
#define MIN_LON_DEG -1800000000LL //!< Minimum Longitude: -180°
#define MAX_LON_DEG 1800000000LL  //!< Maximum Longitude:  180°

// Error codes for some HELPER functions
#define HELPER_ERR_ARG_TOO_SHORT -1    //!< The provided argument is too short
#define HELPER_ERR_OUT_OF_MEMORY -2    //!< The system could not allocate any more memory
#define HELPER_ERR_INVALID_ARGUMENT -3 //!< The argument is invalid
#define HELPER_ERR_UNKNOWN_FORMAT -4   //!< The processed data is of an unknown/unexpected format
#define HELPER_ERR_CHECKSUM_UBX -5     //!< The ubx checksum is invalid
#define HELPER_ERR_ENTRY_NOT_ALLOWED                                                               \
  -6 //!< The processed data contains an valid entry which is not allowed in this context
#define HELPER_ERR_INVALID_MAGIC -7 //!< The magic code of the processed data is wrong
#define HELPER_ERR_UNSUPPORTED_VER                                                                 \
  -8 //!< The data format of the processed data has an unsupported version
#define HELPER_ERR_INVALID_ENTRY_NUM                                                               \
  -9 //!< The processed data contains an invalid number of entries
#define HELPER_ERR_ENTRY_TOO_LONG                                                                  \
  -10 //!< The processed data contains an entry which is longer than supported
#define HELPER_ERR_ENTRY_TOO_SHORT                                                                 \
  -11 //!< The processed data contains an entry which is shorter than supported
#define HELPER_ERR_INVALID_ENTRY -12        //!< The processed data contains an invalid entry
#define HELPER_ERR_BUFFER_SIZE_EXCEEDED -13 //!< The required buffer size exceeds the maximum
#define HELPER_ERR_UNKNOWN_ERROR -100       //!< An unknown error occurred

// Other constants
#define MAX_LOG_LENGTH 4096 //!< Defined the maxium length of log messages

// SOCK_NONBLOCK seems not to be defined in socket.h
// O_NONBLOCK should however work either way
#ifndef SOCK_NONBLOCK
#define SOCK_NONBLOCK O_NONBLOCK //!< If SOCK_NONBLOCK is not defined, O_NONBLOCK is used instead
#endif                           //SOCK_NONBLOCK

//! Indicates the priority of logged message
typedef enum { DEBUG, INFO, WARN, ERROR } LOG_PRIO_t;

//! Logging function pointer
/*! Function pointer type which can be used to
    log messages into the logging facility of
    the caller
    \param prio            : Priority of the log message of
                             the type \ref LOG_PRIO_t
    \param msg             : '\0' terminated log string
*/
typedef void (*LOG_FUNC_p)(LOG_PRIO_t, char const *msg);

//! write()-style Function pointer
/*! Function pointer type which imitates the
    write()-signature with the difference that
    functions pointet to by this type of pointer
    will also accept a context variable.
    \param context         : User selectable content
    \param buf             : Buffer to be written
    \param size            : Number of bytes in the buffer
                             (Must be smaller than SSIZE_MAX)
    \return                  Number of bytes written on
                             success and a negative value
                             on failure.
*/
typedef ssize_t (*WRITE_FUNC_p)(void const *context, unsigned char const *buf, size_t size);

//! Indicates the result of a function
typedef enum {
  SUCCESS,   //!< This function succeeded
  FAIL,      //!< This function failed
  CALL_AGAIN //!< Call this function again
} TRI_STATE_FUNC_RESULT_t;

//! A buffer reference
typedef struct BUF_s
{
  unsigned char *p; //!< The pointer to the buffer
  size_t i;         //!< The number of bytes in \ref p
} BUF_t;

//! A constant buffer reference
typedef struct BUFC_s
{
  unsigned char const *p; //!< The constant pointer to the buffer
  size_t i;               //!< The number of bytes in \ref p
} BUFC_t;

//! UBX Message type identificator
struct UBX_MSG_TYPE
{
  U1 clsId; //!< The class ID
  U1 msgId; //!< The message ID
};

//! Structure of a UBX-MGA-INI-POS message
typedef struct
{
  unsigned char pay[24]; //!< Payload of the message
} MGA_INI_TIME_UTC_t;

//! Structure of a UBX-MGA-INI-POS message
typedef struct
{
  unsigned char pay[20]; //!< Payload of the message
} MGA_INI_POS_t;

//! A time value including an accuracy estimation
typedef struct
{
  struct timespec time; //!< A time
  struct timespec acc;  //!< Accuracy of the \ref time value
  bool leapSeconds;     //!< If true, \ref time includes leapseconds
  bool valid;           //!< Are the contents of this struct valid?
} ACCTIME_t;

//! An accurate time including a creation time
typedef struct
{
  struct timespec stamp; //!< The point in time when \ref accTime was added
  ACCTIME_t accTime;     //!< An accurate time value
} STAMPED_TIME_t;

//! A position including accuracy information
typedef struct
{
  I4 latDeg;   //!< Latitude in degrees *e7 (1 deg -> 1e7)
  I4 lonDeg;   //!< Longitude in degrees *e7 (1 deg -> 1e7)
  U4 posAccCm; //!< Accuracy of the \ref latDeg and lonDeg in cm
  bool valid;  //!< Are the contents of this struct valid?
} POS_t;

//! An accurate position including creation time
typedef struct
{
  ACCTIME_t storeTime; //!< The point in time when \ref pos was added
  POS_t pos;           //!< An accurate position
} STAMPED_POS_t;

//! Struture of the Assistnow Offline / Almanac Plus data
typedef struct
{
  U4 magic;         //!< magic word must be 0x015062b5
  U2 reserved0[32]; //!< Reserved block 0
  U2 size;          //!< size of full file, incl. header, in units of 4 bytes
  U2 reserved1;     //!< Reserved block 1
  U4 reserved2;     //!< Reserved block 2
  U4 p_tow;         //!< Start of prediction Time of week
  U2 p_wno;         //!< Start of prediction Weeknumber
  U2 duration;      //!< Nominal/Typical duration of predictions, in units of 10 minutes.
} ALP_FILE_HEADER_t;

//! GPS Time
typedef struct
{
  struct timespec tow; //!< Time of week
  I4 wno;              //!< Weeknumber since last rollover
  I4 ro;               //!< Number of rollovers since GPS Epoch
} GPS_TIME_t;

#endif //HELPER_TYPES
