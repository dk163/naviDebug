/******************************************************************************
 * Copyright (C) u-blox AG
 * u-blox AG, Thalwil, Switzerland
 *
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice
 * is included in all copies of any software which is or includes a copy
 * or modification of this software and in all copies of the supporting
 * documentation for such software.
 *
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
 * OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 ******************************************************************************
 *
 * Project: libMGA
 * Purpose: Generic command line argument parser.
 *
 *****************************************************************************/

#ifndef __MGA_LIB_ARGUMENTS_H__
#define __MGA_LIB_ARGUMENTS_H__

#include "CommonTypes.h"

//! command line arguments
typedef struct
{
    // interface configuration
    const char*     port;               //!< COM port name
    unsigned int    baudrate;           //!< Baud rate for serial port
    // server configuration
    const char*     server1;            //!< MGA server 1 address
    const char*     server2;            //!< MGA server 2 address
    const char*     token;              //!< MGA token
    const char*     gnss;               //!< GNSS systems, comma separated
    const char*     dataType;           //!< Data type
    const char*     format;             //!< Data format
    const char*     alm;                //!< GNSS systems from which to get almanac data, comma separated
    // online options
    unsigned int    usePosition;        //!< Use reference position
    double          posAccuracy;        //!< Position accuracy in m
    double          posAltitude;        //!< Altitude
    double          posLatitude;        //!< Latitude
    double          posLongitude;       //!< Longitude
    unsigned int    useLatency;         //!< Use latency
    double          latency;            //!< Latency in s
    unsigned int    useTimeAccuracy;    //!< Use time accuracy
    double          timeAccuracy;       //!< Time accuracy in s
    // offline options
    unsigned int    period;             //!< Number of weeks into the future the data should be valid for
    unsigned int    resolution;         //!< Resolution of the data
    unsigned int    days;               //!< The number of days into the future the data should be valid for
    // test configuration
    const char*     test;               //!< MGA test to perform
    // flow control configuration
    unsigned int    timeout;            //!< Timeout
    unsigned int    retry;              //!< Retry count
    const char*     flow;               //!< Flow control
    // legacy configuration
    int             legacyServerDuration; //!< Legacy server duration
    // general configuration
    unsigned int    verbosity;          //!< Verbosity level

    //server certificate verification
    int serverVerify;                  //!< Verify server certificate
} CL_ARGUMENTS_t;
typedef CL_ARGUMENTS_t *CL_ARGUMENTS_pt;//!< pointer to CL_ARGUMENTS_t type

//! defaults if no command line arguments provided
const CL_ARGUMENTS_t defaultArgs =
{
#ifdef _WIN32
    "\\\\.\\COM1",                      //!< COM port on Windows
#else
    "/dev/ttyACM0",                     //!< COM port on Linux
#endif // _WIN32
    9600,                               //!< Baud rate
    "online-live1.services.u-blox.com", //!< MGA server 1 name
    "",                                 //!< MGA server 2 name
    "",                                 //!< MGA token
    "gps",                              //!< GNSS systems, comma separated
    "eph",                              //!< Data type
    "mga",                              //!< Data format
    "gps",                              //!< GNSS systems from which to get almanac data, comma separated
    0,                                  //!< Use reference position (0 or 1)
    0,                                  //!< Position accuracy in m
    0,                                  //!< Altitude
    0,                                  //!< Latitude
    0,                                  //!< Longitude
    0,                                  //!< Use latency (0 or 1)
    0,                                  //!< Latency in s
    0,                                  //!< Use time accuracy
    0,                                  //!< Time accuracy in s
    4,                                  //!< Period
    1,                                  //!< Resolution
    28,                                 //!< Days (28 for mga, 14 for aid)
    "online",                           //!< MGA test to perform
    2000,                               //!< Timeout
    2,                                  //!< Retry count
    "smart",                            //!< Flow control
    24 * 3600,                            //!< Offline host server duration in s
    0,                                  //!< Verbosity level
    0,                                  //!< Server certificate verification
};

//! argument enumeration
typedef enum
{
    UNKNOWN_ARG = 0,                    //!< Unknown argument
    HELP,                               //!< Help request
    PORT,                               //!< COM port
    BAUDRATE,                           //!< Baud rate
    SERVER1,                            //!< MGA server 1 address
    SERVER2,                            //!< MGA server 2 address
    TOKEN,                              //!< MGA token
    GNSS,                               //!< GNSS systems, comma separated
    DATATYPE,                           //!< Data type
    FORMAT,                             //!< Data format
    ALMANAC,                            //!< GNSS systems from which to get almanac data, comma separated
    USEPOSITION,                        //!< Use reference position
    ACCURACY,                           //!< Position accuracy in m
    ALTITUDE,                           //!< Altitude
    LATITUDE,                           //!< Latitude
    LONGITUDE,                          //!< Longitude
    USELATENCY,                         //!< Use latency (0 or 1)
    LATENCY,                            //!< Latency in s
    USETIMEACCURACY,                    //!< Use time accuracy
    TIMEACCURACY,                       //!< Time accuracy in s
    PERIOD,                             //!< Number of weeks into the future the data should be valid for
    RESOLUTION,                         //!< Resolution of the data
    DAYS,                               //!< The number of days into the future the data should be valid for
    TEST,                               //!< MGA test to perform
    TIMEOUT,                            //!< Timeout
    RETRY,                              //!< Retry count
    FLOW,                               //!< Flow control
    VERBOSITY,                          //!< Verbosity level
    LEGACYHOSTDUR,                      //!< Duration to run legacy offline host based server (in seconds)
    SERVERVERIFY,                       //!< Use server certificate verification (1 or 0)
} ARG_t;

//! arguments association
typedef struct
{
    const char* argstr;                 //!< argument string from command line
    ARG_t       argass;                 //!< associated argument type
    const char* argdesc;                //!< argument short description
} ARG_ASSOC_t;

//! known arguments and according identifier
static ARG_ASSOC_t knownArgs[] =
{
    { "-help", HELP, "Show help" },
    { "-token", TOKEN, "MGA token (required)" },
    { "-test", TEST, "MGA test (required)" },
    { "-b", BAUDRATE, "Set baud rate in bit/s" },
    { "-p", PORT, "Set COM port" },
    { "-s1", SERVER1, "IP address/name of server 1" },
    { "-s2", SERVER2, "IP address/name of server 2" },
    { "-gnss", GNSS, "GNSS systems, comma separated" },
    { "-timeout", TIMEOUT, "Timeout in ms" },
    { "-data", DATATYPE, "Data type, comma separated" },
    { "-format", FORMAT, "Data format" },
    { "-alm", ALMANAC, "GNSS systems from which to get almanac data, comma separated" },
    { "-retry", RETRY, "Retry count" },
    { "-flow", FLOW, "Flow control" },
    { "-usepos", USEPOSITION, "Use position (0 or 1)" },
    { "-accuracy", ACCURACY, "Position accuracy in m" },
    { "-altitude", ALTITUDE, "Position altitude" },
    { "-latitude", LATITUDE, "Position latitude" },
    { "-longitude", LONGITUDE, "Position longitude" },
    { "-uselat", USELATENCY, "Use latency (0 or 1)" },
    { "-latency", LATENCY, "Latency in s" },
    { "-useTacc", USETIMEACCURACY, "Use time accuracy (0 or 1)" },
    { "-tacc", TIMEACCURACY, "Time accuracy in s" },
    { "-period", PERIOD, "The number of weeks into the future the data should be valid for" },
    { "-resolution", RESOLUTION, "The resolution of the data" },
    { "-days", DAYS, "The number of days into the future the data should be valid for" },
    { "-serverduration", LEGACYHOSTDUR, "Duration to run legacy offline host based server (in seconds)" },
    { "-serververify", SERVERVERIFY, "Use server certificate verification (1 or 0)" },
    { "-v", VERBOSITY, "Verbosity level (0 or 1)" },
};

//! function to parse the arguments
bool parseArguments(int argc, char *argv[], CL_ARGUMENTS_pt clargs);

//! function to display the usage
void printUsage(void);

// function to display the used configuration
void printConfiguration(CL_ARGUMENTS_pt clargs);

//! function to get the argument association
ARG_t getArgAssociation(char *arg);

//! function to set the program options
bool setArgument(ARG_t option, char *value, CL_ARGUMENTS_pt clargs);

#endif // __MGA_LIB_ARGUMENTS_H__
